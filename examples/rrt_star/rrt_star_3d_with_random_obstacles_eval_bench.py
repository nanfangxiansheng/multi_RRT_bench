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
import heapq
import time
X_dimensions = np.array([(0, 100), (0, 100), (0, 100)])  # dimensions of Search Space
x_init = (0, 0, 0)  # starting location
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

def test_algorithm(algorithm_class,X, q, x_init, x_goal, max_samples, r, **kwargs):
    print(X)
    # create rrt_search
    start_rrt_star=time.time()
    
    # 根据算法类型创建实例
    if algorithm_class==RRTStar:
        rrt_star = algorithm_class(X, q, x_init, x_goal, max_samples, r, prc, rewire_count)
        path_rrt = rrt_star.rrt_star()

    elif algorithm_class==RRTConnect:
        rrt_star = algorithm_class(X, q, x_init, x_goal, max_samples, r, prc)
        path_rrt = rrt_star.rrt_connect()

    elif algorithm_class==RRTStarBidirectionalHeuristic:
        rrt_star = algorithm_class(X, q, x_init, x_goal, max_samples, r, prc,rewire_count)
        path_rrt = rrt_star.rrt_star_bid_h()

    elif algorithm_class==RRTStarBidirectional:
        rrt_star = algorithm_class(X, q, x_init, x_goal, max_samples, r, prc)
        path_rrt = rrt_star.rrt_star_bidirectional()

    elif algorithm_class==RRT:
        rrt_star = algorithm_class(X, q, x_init, x_goal, max_samples, r, prc)
        path_rrt = rrt_star.rrt_search()

    else:
        raise ValueError(f"Unsupported algorithm class: {algorithm_class}")

    end_rrt_star=time.time()

    if path_rrt is not None:
        nodes_number=len(path_rrt)
        path_length=find_total_length_of_path(path_rrt)
        cost_time=end_rrt_star-start_rrt_star
        print(f"time cost of {algorithm_class.__name__}:",end_rrt_star-start_rrt_star)
        print(f"path length of {algorithm_class.__name__}:",find_total_length_of_path(path_rrt))
        return path_length,cost_time,nodes_number
    else:
        return None,None,None

def test_all():
    n = 50
    X = SearchSpace(X_dimensions)

    x_init = (0, 0, 0)  # starting location
    x_goal = (60+random.randint(1,10), 30+random.randint(1,20), 100)  # goal location
    Obstacles = generate_random_obstacles(X, x_init, x_goal, n)
    test_cost_path_this_epoch=[]
    test_cost_time_this_epoch=[]
    test_nodes_number=[]

    # 包含A*算法在内的所有算法
    algorithms = [RRTStar, RRTConnect, RRTStarBidirectionalHeuristic, RRT]
    
    for algorithm_class in algorithms:
        path_len,time_cost,nodes_number=test_algorithm(algorithm_class,X, q, x_init, x_goal, max_samples, r)
        test_cost_path_this_epoch.append(path_len)
        test_cost_time_this_epoch.append(time_cost)
        test_nodes_number.append(nodes_number)

    return test_cost_path_this_epoch,test_cost_time_this_epoch,test_nodes_number

def main():
    test_epoch=20
    total_count_path=[]
    total_count_time=[]
    total_count_nodes=[]
    test_classes=4 # 更新算法数量
    all_success_path=[]
    all_success_time=[]
    all_success_nodes=[]
    
    def validate_all_success(path:list):
        for j in path:
            if j is None:
                return False
        return True

    for i in range(test_epoch):
        path,time,nodes=test_all()
        total_count_path.append(path)
        total_count_time.append(time)
        total_count_nodes.append(nodes)
        if validate_all_success(path):
            all_success_path.append(path)
            all_success_time.append(time)
            all_success_nodes.append(nodes)
        
        print(f"epoch:{i} begins:")
    
    success_counts=[0]*test_classes
    for i in range(len(total_count_path)):
        for j in range(test_classes):
            if total_count_path[i][j] is not None:
                success_counts[j]+=1
    
    print(f"success_counts:{success_counts}")
    
    # 转换为numpy数组进行统计
    all_success_path=np.array(all_success_path)
    all_success_time=np.array(all_success_time)
    all_success_nodes=np.array(all_success_nodes)
    
    # 算法名称列表（包含A*）
    name_list=["RRTStar", "RRTConnect", "RRTStarBidirectionalHeuristic", "RRT"]
    
    print(name_list)
    
    if len(all_success_path) > 0:
        print(f"all_success_path:{all_success_path.mean(axis=0)}")
        print(f"all_success_time:{all_success_time.mean(axis=0)}")
        print(f"all_success_nodes:{all_success_nodes.mean(axis=0)}")
        
        # 计算相对性能
        avg_times = all_success_time.mean(axis=0)
        if len(avg_times) > 1:
            baseline_time = avg_times[0]  # 以RRTStar为基准
            relative_performance = baseline_time / avg_times
            print(f"relative_performance (vs RRTStar):{relative_performance}")
    else:
        print("没有找到任何成功的路径，无法计算统计数据")

if __name__ == '__main__':
    main()

# plot
# plot = Plot("rrt_star_3d_with_random_obstacles")
# plot.plot_tree(X, rrt.trees)
# if path is not None:
#     plot.plot_path(X, path)
# plot.plot_obstacles(X, Obstacles)

# plot.plot_start(X, x_init)
# plot.plot_goal(X, x_goal)
# plot.draw(auto_open=True)