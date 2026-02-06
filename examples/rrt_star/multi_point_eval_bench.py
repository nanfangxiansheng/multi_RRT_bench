# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np
from rrt_algorithms.utilities.geometry import dist_between_points
from rrt_algorithms.rrt.rrt_star import RRTStar
from rrt_algorithms.search_space.search_space import SearchSpace
from rrt_algorithms.utilities.obstacle_generation import generate_random_obstacles
from rrt_algorithms.utilities.plotting import Plot
from rrt_algorithms.rrt.rrt_connect import RRTConnect
from rrt_algorithms.rrt.rrt_star_bid_h import RRTStarBidirectionalHeuristic
from rrt_algorithms.rrt.rrt_star_bid import RRTStarBidirectional
from rrt_algorithms.rrt.rrt import RRT
import random
import time
X_dimensions = np.array([(0, 100), (0, 100), (0, 100)])  # dimensions of Search Space
x_init = (0, 0, 0)  # starting location
x_pickup = (30, 50, 50)  # pickup location (新增取货点)
x_goal = (60, 30, 100)  # goal location
from typing import List, Tuple, Optional

def find_total_length_of_path(path:list=None):
    return sum([dist_between_points(path[i], path[i+1]) for i in range(len(path)-1)])

q = 8  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples =1024  # max number of samples to take before timing out
rewire_count = 32  # optional, number of nearby branches to rewire
prc = 0.1  # probability of checking for a connection to goal
# create Search Space

def rrt_search_with_pickup(algorithm_class, X, q, x_init, x_pickup, x_goal, max_samples, r, **kwargs):
    """
    从起点到取货点再到终点的RRT搜索
    """
    total_path = []
    total_time = 0
    
    # 第一段：起点到取货点
    print(f"Planning from start to pickup point...")
    start_time = time.time()
    
    if algorithm_class == RRTStar:
        rrt1 = algorithm_class(X, q, x_init, x_pickup, max_samples, r, prc, rewire_count)
        path1 = rrt1.rrt_star()
    elif algorithm_class == RRTConnect:
        rrt1 = algorithm_class(X, q, x_init, x_pickup, max_samples, r, prc)
        path1 = rrt1.rrt_connect()
    elif algorithm_class == RRTStarBidirectionalHeuristic:
        rrt1 = algorithm_class(X, q, x_init, x_pickup, max_samples, r, prc, rewire_count)
        path1 = rrt1.rrt_star_bid_h()
    elif algorithm_class == RRTStarBidirectional:
        rrt1 = algorithm_class(X, q, x_init, x_pickup, max_samples, r, prc)
        path1 = rrt1.rrt_star_bidirectional()
    elif algorithm_class == RRT:
        rrt1 = algorithm_class(X, q, x_init, x_pickup, max_samples, r, prc)
        path1 = rrt1.rrt_search()
    else:
        raise ValueError(f"Unsupported algorithm class: {algorithm_class}")
    
    time1 = time.time() - start_time
    total_time += time1
    
    if path1 is None:
        print("Failed to find path from start to pickup point")
        return None, 0, 0
    
    # 第二段：取货点到终点
    print(f"Planning from pickup point to goal...")
    start_time = time.time()
    
    if algorithm_class == RRTStar:
        rrt2 = algorithm_class(X, q, x_pickup, x_goal, max_samples, r, prc, rewire_count)
        path2 = rrt2.rrt_star()
    elif algorithm_class == RRTConnect:
        rrt2 = algorithm_class(X, q, x_pickup, x_goal, max_samples, r, prc)
        path2 = rrt2.rrt_connect()
    elif algorithm_class == RRTStarBidirectionalHeuristic:
        rrt2 = algorithm_class(X, q, x_pickup, x_goal, max_samples, r, prc, rewire_count)
        path2 = rrt2.rrt_star_bid_h()
    elif algorithm_class == RRTStarBidirectional:
        rrt2 = algorithm_class(X, q, x_pickup, x_goal, max_samples, r, prc)
        path2 = rrt2.rrt_star_bidirectional()
    elif algorithm_class == RRT:
        rrt2 = algorithm_class(X, q, x_pickup, x_goal, max_samples, r, prc)
        path2 = rrt2.rrt_search()
    
    time2 = time.time() - start_time
    total_time += time2
    
    if path2 is None:
        print("Failed to find path from pickup point to goal")
        return None, 0, 0
    
    # 合并两段路径（去掉取货点的重复点）
    total_path = path1[:-1] + path2
    total_nodes = len(total_path)
    
    return total_path, total_time, total_nodes

def test_algorithm_with_pickup(algorithm_class, X, q, x_init, x_pickup, x_goal, max_samples, r, **kwargs):
    """
    测试带取货点的算法
    """
    start_time = time.time()
    
    # 使用带取货点的RRT搜索
    path_rrt, segment_time, nodes_number = rrt_search_with_pickup(
        algorithm_class, X, q, x_init, x_pickup, x_goal, max_samples, r, **kwargs
    )
    
    end_time = time.time()
    
    if path_rrt is not None:
        path_length = find_total_length_of_path(path_rrt)
        cost_time = end_time - start_time
        
        print(f"time cost of {algorithm_class.__name__} (with pickup):", cost_time)
        print(f"path length of {algorithm_class.__name__} (with pickup):", path_length)
        print(f"path nodes of {algorithm_class.__name__} (with pickup):", nodes_number)
        
        return path_length, cost_time, nodes_number, path_rrt
    else:
        return None, None, None, None

def test_all_with_pickup():
    """
    测试所有算法（带取货点）
    """
    n = 50
    X = SearchSpace(X_dimensions)

    x_init = (0, 0, 0)  # starting location
    x_pickup = (30 + random.randint(1, 10), 50 + random.randint(1, 20), 50 + random.randint(1, 20))  # pickup location
    x_goal = (60 + random.randint(1, 10), 30 + random.randint(1, 20), 100)  # goal location
    
    # 生成障碍物
    Obstacles = generate_random_obstacles(X, x_init, x_goal, n)
    
    test_cost_path_this_epoch = []
    test_cost_time_this_epoch = []
    test_nodes_number = []
    test_paths = []  # 保存路径用于可视化
    
    # 测试的算法
    algorithms = [RRTStar, RRTConnect, RRTStarBidirectionalHeuristic, RRT]
    
    for algorithm_class in algorithms:
        path_len, time_cost, nodes_number, path = test_algorithm_with_pickup(
            algorithm_class, X, q, x_init, x_pickup, x_goal, max_samples, r
        )
        test_cost_path_this_epoch.append(path_len)
        test_cost_time_this_epoch.append(time_cost)
        test_nodes_number.append(nodes_number)
        test_paths.append(path)

    return test_cost_path_this_epoch, test_cost_time_this_epoch, test_nodes_number, test_paths, (x_init, x_pickup, x_goal)

def main():
    """
    主函数 - 测试带取货点的路径规划
    """
    test_epoch = 20
    test_classes = 4  # 算法数量
    
    total_count_path = []
    total_count_time = []
    total_count_nodes = []
    all_success_path = []
    all_success_time = []
    all_success_nodes = []
    
    # 保存最后一轮测试的数据用于可视化
    last_test_paths = None
    last_test_waypoints = None
    
    def validate_all_success(path: list):
        for j in path:
            if j is None:
                return False
        return True

    for i in range(test_epoch):
        print(f"\n=== Epoch {i+1}/{test_epoch} ===")
        path, time_cost, nodes, paths, waypoints = test_all_with_pickup()
        
        total_count_path.append(path)
        total_count_time.append(time_cost)
        total_count_nodes.append(nodes)
        
        # 保存最后一轮数据
        if i == test_epoch - 1:
            last_test_paths = paths
            last_test_waypoints = waypoints
        
        if validate_all_success(path):
            all_success_path.append(path)
            all_success_time.append(time_cost)
            all_success_nodes.append(nodes)
        
        print(f"epoch:{i+1} completed")
    
    # 统计成功率
    success_counts = [0] * test_classes
    for i in range(len(total_count_path)):
        for j in range(test_classes):
            if total_count_path[i][j] is not None:
                success_counts[j] += 1
    
    print(f"\n=== Results Summary ===")
    print(f"Success counts: {success_counts}")
    
    # 算法名称
    name_list = ["RRTStar", "RRTConnect", "RRTStarBidirectionalHeuristic", "RRT"]
    
    # 打印详细统计
    for i in range(test_classes):
        print(f"\n{name_list[i]}:")
        print(f"  Success rate: {success_counts[i]}/{test_epoch} ({success_counts[i]/test_epoch*100:.1f}%)")
        
        # 收集成功的数据
        success_paths = []
        success_times = []
        success_nodes = []
        
        for epoch in range(test_epoch):
            if total_count_path[epoch][i] is not None:
                success_paths.append(total_count_path[epoch][i])
                success_times.append(total_count_time[epoch][i])
                success_nodes.append(total_count_nodes[epoch][i])
        
        if success_paths:
            avg_path = np.mean(success_paths)
            avg_time = np.mean(success_times)
            avg_nodes = np.mean(success_nodes)
            
            print(f"  Avg path length: {avg_path:.2f}")
            print(f"  Avg time: {avg_time:.2f}s")
            print(f"  Avg nodes: {avg_nodes:.1f}")



if __name__ == "__main__":
    main()