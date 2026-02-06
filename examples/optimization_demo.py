#!/usr/bin/env python3
"""
RRT*算法优化演示脚本
展示原始版本和优化版本的性能对比
"""

import time
import numpy as np
from rrt_algorithms.search_space.search_space import SearchSpace
from rrt_algorithms.utilities.plotting import Plot
from rrt_algorithms.rrt.rrt_star import RRTStar
from rrt_algorithms.rrt.rrt_star_optimized import RRTStarOptimized

def create_test_environment():
    """创建测试环境"""
    # 创建2D搜索空间
    X_dimensions = np.array([(0, 100), (0, 100)])  # 100x100的空间
    
    # 添加一些障碍物
    Obstacles = np.array([
        (20, 20, 40, 40),    # 正方形障碍物
        (60, 10, 80, 30),
        (10, 60, 30, 80),
        (50, 50, 70, 70),
        (80, 80, 90, 90)
    ])
    
    # 起点和终点
    x_init = (10, 10)
    x_goal = (90, 90)
    
    return X_dimensions, Obstacles, x_init, x_goal

def benchmark_algorithm(algorithm_class, X, q, x_init, x_goal, max_samples, r, **kwargs):
    """基准测试函数"""
    start_time = time.time()
    
    # 创建算法实例
    rrt = algorithm_class(X, q, x_init, x_goal, max_samples, r, **kwargs)
    
    # 运行算法
    path = rrt.rrt_star()
    
    end_time = time.time()
    execution_time = end_time - start_time
    
    return path, execution_time, rrt.trees[0].V_count

def main():
    print("=== RRT*算法优化演示 ===\n")
    
    # 创建测试环境
    X_dimensions, Obstacles, x_init, x_goal = create_test_environment()
    X = SearchSpace(X_dimensions, Obstacles)
    
    # 算法参数
    q = 3.0  # 边长
    max_samples = 1000
    r = 1.0  # 分辨率
    
    print("环境设置:")
    print(f"- 搜索空间: {X_dimensions}")
    print(f"- 障碍物数量: {len(Obstacles)}")
    print(f"- 起点: {x_init}")
    print(f"- 终点: {x_goal}")
    print(f"- 最大样本数: {max_samples}")
    print(f"- 边长参数: {q}")
    print(f"- 分辨率: {r}\n")
    
    # 测试原始版本
    print("1. 测试原始RRT*算法...")
    original_path, original_time, original_nodes = benchmark_algorithm(
        RRTStar, X, q, x_init, x_goal, max_samples, r
    )
    
    print(f"   执行时间: {original_time:.4f} 秒")
    print(f"   生成节点数: {original_nodes}")
    print(f"   路径长度: {len(original_path) if original_path else 0}")
    print(f"   路径成本: {calculate_path_cost(original_path):.2f}" if original_path else "   未找到路径")
    
    # 测试优化版本
    print("\n2. 测试优化RRT*算法...")
    optimized_path, optimized_time, optimized_nodes = benchmark_algorithm(
        RRTStarOptimized, X, q, x_init, x_goal, max_samples, r,
        rewire_count=20,  # 限制重连数量
        use_cached_costs=True,  # 使用缓存
        early_termination=True  # 早期终止
    )
    
    print(f"   执行时间: {optimized_time:.4f} 秒")
    print(f"   生成节点数: {optimized_nodes}")
    print(f"   路径长度: {len(optimized_path) if optimized_path else 0}")
    print(f"   路径成本: {calculate_path_cost(optimized_path):.2f}" if optimized_path else "   未找到路径")
    
    # 性能对比
    print("\n=== 性能对比 ===")
    if original_time > 0:
        speedup = original_time / optimized_time if optimized_time > 0 else float('inf')
        print(f"速度提升: {speedup:.2f}x")
    
    node_reduction = (1 - optimized_nodes / original_nodes) * 100 if original_nodes > 0 else 0
    print(f"节点减少: {node_reduction:.1f}%")
    
    # 可视化结果
    print("\n3. 生成可视化结果...")
    plot = Plot("rrt_star_optimization_comparison")
    
    # 绘制原始版本结果
    if original_path:
        plot.plot_tree(X, rrt.trees[0], 'original_tree.png', color='blue')
        plot.plot_path(X, original_path, 'original_path.png', color='red')
        print("   原始版本结果已保存: original_tree.png, original_path.png")
    
    # 重新创建优化版本实例用于可视化
    rrt_opt = RRTStarOptimized(X, q, x_init, x_goal, max_samples, r, 
                              rewire_count=20, use_cached_costs=True, early_termination=True)
    rrt_opt.rrt_star()
    
    if optimized_path:
        plot.plot_tree(X, rrt_opt.trees[0], 'optimized_tree.png', color='green')
        plot.plot_path(X, optimized_path, 'optimized_path.png', color='orange')
        print("   优化版本结果已保存: optimized_tree.png, optimized_path.png")

def calculate_path_cost(path):
    """计算路径总成本"""
    if not path or len(path) < 2:
        return 0
    
    total_cost = 0
    for i in range(len(path) - 1):
        p1, p2 = np.array(path[i]), np.array(path[i + 1])
        total_cost += np.linalg.norm(p2 - p1)
    
    return total_cost

if __name__ == '__main__':
    main()