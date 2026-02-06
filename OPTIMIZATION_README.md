# RRT*算法优化功能使用指南

## 快速开始

### 1. 基本使用

```python
from rrt_algorithms.search_space.search_space import SearchSpace
from rrt_algorithms.rrt.rrt_star_optimized import RRTStarOptimized

# 创建搜索空间
X_dimensions = np.array([(0, 100), (0, 100)])
Obstacles = np.array([(20, 20, 40, 40), (60, 60, 80, 80)])
X = SearchSpace(X_dimensions, Obstacles)

# 创建优化版RRT*
rrt_opt = RRTStarOptimized(
    X=X,
    q=3.0,              # 边长
    x_init=(10, 10),    # 起点
    x_goal=(90, 90),    # 终点
    max_samples=1000,   # 最大样本数
    r=1.0,              # 分辨率
    rewire_count=20,    # 重连计数限制
    use_cached_costs=True,  # 启用成本缓存
    early_termination=True   # 启用早期终止
)

# 运行算法
path = rrt_opt.rrt_star()
```

### 2. 运行演示脚本

```bash
# 运行性能对比演示
python examples/optimization_demo.py

# 运行基准测试
python examples/benchmark_optimization.py
```

## 优化特性详解

### 成本缓存 (Cost Caching)
```python
# 自动缓存路径成本计算结果，避免重复计算
rrt_opt = RRTStarOptimized(..., use_cached_costs=True)
```

### 重连计数限制 (Rewire Count Limiting)
```python
# 限制每次重连操作的顶点数量
rrt_opt = RRTStarOptimized(..., rewire_count=20)  # 默认20个
```

### 早期终止 (Early Termination)
```python
# 当找到满意解时提前终止
rrt_opt = RRTStarOptimized(..., early_termination=True)
```

### 自适应重连 (Adaptive Rewiring)
```python
# 根据搜索进度自动调整重连策略
# 早期阶段更积极重连，后期阶段保守重连
```

## 性能调优建议

### 参数选择指南

| 环境复杂度 | rewire_count | early_termination | 预期加速比 |
|------------|--------------|-------------------|------------|
| 简单       | 10-15        | True              | 2.5-3.0x   |
| 中等       | 20-30        | True              | 2.0-2.5x   |
| 复杂       | 30-50        | False             | 1.5-2.0x   |

### 内存使用优化
```python
# 对于内存受限环境，可以禁用某些缓存
rrt_opt = RRTStarOptimized(
    ...,
    use_cached_costs=True,      # 保留关键缓存
    cache_size_limit=10000      # 限制缓存大小
)
```

## API差异

优化版本与原始版本API基本兼容，新增参数：

```python
# 新增参数
RRTStarOptimized.__init__(
    ...,
    rewire_count=None,      # 重连顶点数量限制
    use_cached_costs=True,  # 是否使用成本缓存
    early_termination=False, # 是否启用早期终止
    cache_size_limit=None,   # 缓存大小限制
    adaptive_rewiring=True   # 是否启用自适应重连
)
```

## 故障排除

### 常见问题

1. **路径质量下降**
   ```python
   # 增加重连计数
   rrt_opt = RRTStarOptimized(..., rewire_count=50)
   ```

2. **内存使用过高**
   ```python
   # 限制缓存大小
   rrt_opt = RRTStarOptimized(..., cache_size_limit=5000)
   ```

3. **收敛速度慢**
   ```python
   # 启用早期终止
   rrt_opt = RRTStarOptimized(..., early_termination=True)
   ```

## 性能监控

```python
# 获取性能统计信息
stats = rrt_opt.get_performance_stats()
print(f"缓存命中率: {stats['cache_hit_rate']:.2%}")
print(f"重连次数: {stats['rewire_operations']}")
print(f"碰撞检测次数: {stats['collision_checks']}")
```

## 贡献指南

欢迎提交Issue和Pull Request来改进优化功能！

主要关注点：
- 新的优化策略
- 性能改进
- 代码质量提升
- 文档完善