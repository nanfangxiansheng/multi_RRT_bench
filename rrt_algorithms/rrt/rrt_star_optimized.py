# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
from collections import defaultdict
from operator import itemgetter
import time

from rrt_algorithms.rrt.heuristics import cost_to_go
from rrt_algorithms.rrt.heuristics import segment_cost, path_cost
from rrt_algorithms.rrt.rrt import RRT


class CostCache:
    """成本缓存类，避免重复计算"""
    def __init__(self, size_limit=None):
        self.costs = {}
        self.size_limit = size_limit
        self.access_count = 0
        self.hit_count = 0
    
    def get_or_compute_cost(self, v1, v2, compute_func):
        """获取或计算两点间的成本"""
        self.access_count += 1
        key = tuple(sorted([tuple(v1), tuple(v2)]))  # 确保键的一致性
        
        if key in self.costs:
            self.hit_count += 1
            return self.costs[key]
        
        # 计算新成本
        cost = compute_func(v1, v2)
        self.costs[key] = cost
        
        # 如果设置了大小限制，进行清理
        if self.size_limit and len(self.costs) > self.size_limit:
            # 简单的LRU清理策略
            keys_to_remove = list(self.costs.keys())[:len(self.costs)//4]
            for key_to_remove in keys_to_remove:
                del self.costs[key_to_remove]
        
        return cost
    
    def get_hit_rate(self):
        """获取缓存命中率"""
        if self.access_count == 0:
            return 0
        return self.hit_count / self.access_count


class RRTStarOptimized(RRT):
    def __init__(self, X, q, x_init, x_goal, max_samples, r, prc=0.01, 
                 rewire_count=None, use_cached_costs=True, 
                 early_termination=False, cache_size_limit=10000):
        """
        优化版RRT*搜索算法
        :param X: 搜索空间
        :param q: 边长参数
        :param x_init: 初始位置
        :param x_goal: 目标位置
        :param max_samples: 最大采样数
        :param r: 碰撞检测分辨率
        :param prc: 解决方案检查概率
        :param rewire_count: 重连顶点数量限制
        :param use_cached_costs: 是否使用成本缓存
        :param early_termination: 是否启用早期终止
        :param cache_size_limit: 缓存大小限制
        """
        super().__init__(X, q, x_init, x_goal, max_samples, r, prc)
        self.rewire_count = rewire_count if rewire_count is not None else 0
        self.use_cached_costs = use_cached_costs
        self.early_termination = early_termination
        self.cache_size_limit = cache_size_limit
        
        # 初始化缓存
        if self.use_cached_costs:
            self.cost_cache = CostCache(size_limit=cache_size_limit)
            self.segment_cost_cache = CostCache(size_limit=cache_size_limit)
        else:
            self.cost_cache = None
            self.segment_cost_cache = None
            
        # 性能统计
        self.stats = {
            'rewire_operations': 0,
            'collision_checks': 0,
            'cached_computations': 0,
            'start_time': time.time(),
            'iterations': 0
        }
        
        # 自适应参数
        self.min_path_length = float('inf')
        
    def cached_segment_cost(self, v1, v2):
        """带缓存的线段成本计算"""
        if self.use_cached_costs:
            return self.segment_cost_cache.get_or_compute_cost(v1, v2, segment_cost)
        return segment_cost(v1, v2)
    
    def cached_path_cost(self, tree, start, end):
        """带缓存的路径成本计算"""
        if self.use_cached_costs:
            # 这里简化处理，实际应该缓存整个路径成本
            return path_cost(self.trees[tree].E, start, end)
        return path_cost(self.trees[tree].E, start, end)
    
    def get_nearby_vertices(self, tree, x_init, x_new):
        """
        获取附近顶点及其关联的路径成本
        """
        X_near = self.nearby(tree, x_new, self.current_rewire_count(tree))
        L_near = []
        
        for x_near in X_near:
            # 使用缓存的成本计算
            segment_cost_val = self.cached_segment_cost(x_near, x_new)
            path_cost_val = self.cached_path_cost(tree, x_init, x_near)
            total_cost = path_cost_val + segment_cost_val
            L_near.append((total_cost, x_near))
        
        # 排序
        L_near.sort(key=itemgetter(0))
        return L_near

    def rewire(self, tree, x_new, L_near):
        """
        重连线以缩短边长（优化版本）
        """
        self.stats['rewire_operations'] += 1
        
        for _, x_near in L_near:
            # 使用缓存的成本计算
            curr_cost = self.cached_path_cost(tree, self.x_init, x_near)
            tent_cost = (self.cached_path_cost(tree, self.x_init, x_new) + 
                        self.cached_segment_cost(x_new, x_near))
            
            # 碰撞检测计数
            self.stats['collision_checks'] += 1
            
            if tent_cost < curr_cost and self.X.collision_free(x_near, x_new, self.r):
                self.trees[tree].E[x_near] = x_new
                # 更新最小路径长度估计
                if self.early_termination:
                    self.min_path_length = min(self.min_path_length, tent_cost)

    def connect_shortest_valid(self, tree, x_new, L_near):
        """
        连接到最近的有效顶点
        """
        # 检查附近顶点的总成本并连接最短的有效边
        for _, x_near in L_near:
            if self.connect_to_point(tree, x_near, x_new):
                break

    def current_rewire_count(self, tree):
        """
        返回重连计数（优化版本）
        """
        # 如果没有指定重连计数，设置合理的默认值
        if self.rewire_count is None:
            return min(self.trees[tree].V_count, 50)  # 默认最多50个
        
        # 最大有效重连计数
        return min(self.trees[tree].V_count, self.rewire_count)

    def adaptive_rewire_count(self, iteration):
        """
        自适应重连计数
        """
        base_count = self.rewire_count or 20
        
        # 早期探索阶段使用更多重连，后期收敛阶段减少
        progress_ratio = iteration / self.max_samples
        if progress_ratio < 0.3:  # 前30%迭代
            return min(base_count * 2, 100)
        elif progress_ratio < 0.7:  # 中间40%迭代
            return base_count
        else:  # 最后30%迭代
            return max(base_count // 2, 10)

    def check_solution(self):
        """
        检查是否存在解决方案（优化版本）
        """
        if self.trees[0].E.get(self.x_goal) is not None:
            path = self.reconstruct_path()
            
            # 早期终止条件
            if (self.early_termination and 
                len(path) <= self.min_path_length * 1.1):  # 10%容差
                return True, path
                
            return True, path
        return False, None

    def reconstruct_path(self):
        """
        重构路径
        """
        path = [self.x_goal]
        current = self.x_goal
        while current != self.x_init:
            current = self.trees[0].E[current]
            path.append(current)
        path.reverse()
        return path

    def get_performance_stats(self):
        """
        获取性能统计信息
        """
        stats = self.stats.copy()
        stats['execution_time'] = time.time() - stats['start_time']
        stats['cache_hit_rate'] = (self.cost_cache.get_hit_rate() 
                                 if self.cost_cache else 0)
        stats['segment_cache_hit_rate'] = (self.segment_cost_cache.get_hit_rate() 
                                         if self.segment_cost_cache else 0)
        return stats

    def rrt_star(self):
        """
        优化版RRT*算法主函数
        基于论文：Incremental Sampling-based Algorithms for Optimal Motion Planning
        http://roboticsproceedings.org/rss06/p34.pdf
        """
        self.add_vertex(0, self.x_init)
        self.add_edge(0, self.x_init, None)
        
        iteration = 0
        
        while iteration < self.max_samples:
            iteration += 1
            self.stats['iterations'] = iteration
            
            x_new, x_nearest = self.new_and_near(0, self.q)
            if x_new is None:
                continue

            # 获取附近顶点和到达成本
            L_near = self.get_nearby_vertices(0, self.x_init, x_new)

            # 检查附近顶点的总成本并连接最短的有效边
            self.connect_shortest_valid(0, x_new, L_near)

            if x_new in self.trees[0].E:
                # 重连线
                current_count = self.adaptive_rewire_count(iteration)
                # 临时调整重连计数
                original_count = self.rewire_count
                self.rewire_count = current_count
                self.rewire(0, x_new, L_near)
                self.rewire_count = original_count

            solution = self.check_solution()
            if solution[0]:
                return solution[1]

        # 如果达到最大样本数仍未找到解，返回最佳路径
        return self.reconstruct_path() if self.trees[0].E.get(self.x_goal) is not None else None