# 火箭回收算法项目

## 项目概述

本项目实现了火箭垂直着陆回收算法，模拟SpaceX Falcon 9火箭的垂直着陆过程。算法基于最优控制理论和PID控制，包含完整的控制系统架构。

## 文件结构

```
rocket_landing/
├── rocket_landing_algorithm.py    # 完整算法实现（需要numpy, matplotlib, scipy）
├── rocket_landing_simple.py       # 简化版本（需要numpy, matplotlib）
├── rocket_landing_test.py         # 测试版本（纯Python）
├── rocket_landing_improved.py     # 改进版本（纯Python）
├── rocket_landing_basic.py        # 基础版本（纯Python）
├── rocket_landing_demo.py         # 演示版本（纯Python）
├── rocket_landing_document.md     # 详细设计文档
├── rocket_landing_summary.md      # 项目总结
└── README.md                      # 本文件
```

## 算法特点

### 1. 控制架构
- **PID控制器**: 实时姿态和推力控制
- **轨迹规划器**: 模型预测控制(MPC)
- **燃料管理器**: 优化燃料使用
- **紧急处理**: 异常情况处理

### 2. 控制策略
- **水平控制**: 通过姿态角调整水平推力
- **垂直控制**: 通过节流阀调整推力大小
- **姿态控制**: 通过摆角调整推力方向
- **分段控制**: 适应不同阶段需求

### 3. 性能指标
- 着陆精度: < 5m
- 着陆速度: < 0.5m/s
- 姿态误差: < 2°
- 燃料效率: 优化控制减少燃料消耗

## 使用方法

### 1. 运行演示版本（推荐）
```bash
py rocket_landing_demo.py
```

### 2. 运行测试版本
```bash
py rocket_landing_test.py
```

### 3. 运行改进版本
```bash
py rocket_landing_improved.py
```

### 4. 运行基础版本
```bash
py rocket_landing_basic.py
```

### 5. 运行完整版本（需要安装依赖）
```bash
pip install numpy matplotlib scipy
py rocket_landing_algorithm.py
```

## 算法设计

### 数学模型

#### 状态变量
```
状态向量: X = [x, y, vx, vy, θ, ω, m, fuel]
```

#### 控制变量
```
控制向量: U = [throttle, gimbal]
```

#### 动力学方程
```
ax = (F * sin(θ + g)) / m
ay = (F * cos(θ + g)) / m - g
α = (F * g * L/2) / I
dm/dt = -F / (Isp * g0)
```

### 控制策略

#### PID控制器
```python
# 水平控制
desired_theta = -Kp_x * dx - Kd_x * dvx

# 垂直控制
base_thrust = mass * g + Kp_y * dy + Kd_y * dvy
throttle = base_thrust / thrust_max

# 姿态控制
gimbal = (desired_theta - actual_theta) * Kp_theta
```

#### 分段控制
- **远程阶段**: 快速接近，大推力
- **中程阶段**: 精确调整，优化轨迹
- **着陆阶段**: 稳定控制，小摆角

## 测试场景

### 场景1: 标准着陆
- 初始高度: 5000m
- 初始水平距离: 1000m
- 初始速度: vx=-50m/s, vy=-100m/s

### 场景2: 高精度着陆
- 初始高度: 3000m
- 初始水平距离: 500m

### 场景3: 简化着陆
- 初始高度: 2000m
- 初始水平距离: 500m

## 性能分析

### 当前状态
- 算法已实现完整控制逻辑
- 控制策略清晰，架构合理
- 需要进一步优化PID参数
- 需要添加轨迹规划算法

### 改进方向
1. **参数整定**: 使用优化算法调整PID参数
2. **轨迹规划**: 实现MPC或最优控制
3. **模型完善**: 添加更多物理约束
4. **鲁棒性增强**: 处理不确定性

## 应用场景

### 1. 航天器回收
- 火箭垂直着陆
- 飞船精准对接
- 卫星回收

### 2. 无人机着陆
- 精准垂直着陆
- 移动平台着陆
- 复杂环境着陆

### 3. 机器人控制
- 精准定位
- 姿态控制
- 路径规划

### 4. 自动驾驶
- 车辆精准停车
- 自动泊车
- 紧急制动

## 扩展方向

### 1. 高级控制算法
- **模型预测控制(MPC)**: 更精确的预测
- **自适应控制**: 参数在线辨识
- **鲁棒控制**: 抗干扰能力
- **智能控制**: 神经网络、强化学习

### 2. 环境感知
- **视觉着陆**: 目标识别与跟踪
- **地形适应**: 不平坦地面着陆
- **风场估计**: 风扰补偿
- **障碍物避障**: 动态环境

### 3. 多火箭协同
- **编队控制**: 多火箭同时着陆
- **任务分配**: 不同着陆点分配
- **资源优化**: 燃料共享
- **通信协调**: 分布式控制

### 4. 硬件在环
- **实时仿真**: 与实际控制器对接
- **故障注入**: 测试鲁棒性
- **硬件测试**: 实际硬件验证
- **飞行测试**: 真实环境验证

## 文档说明

### rocket_landing_document.md
详细的设计文档，包含：
- 问题分析
- 算法架构
- 控制策略
- 实现细节
- 性能指标
- 测试场景
- 扩展方向

### rocket_landing_summary.md
项目总结，包含：
- 算法设计
- 性能指标
- 测试结果
- 问题分析
- 改进方向
- 应用场景

## 技术栈

### 编程语言
- Python 3.x

### 依赖库（可选）
- numpy: 数值计算
- matplotlib: 绘图
- scipy: 优化算法

### 开发环境
- Windows/Linux/macOS
- 任何Python IDE或文本编辑器

## 贡献指南

1. 阅读设计文档了解算法架构
2. 运行演示版本了解算法行为
3. 修改参数进行实验
4. 提交改进和优化

## 许可证

本项目仅供学习和研究使用。

## 联系方式

如有问题或建议，请通过GitHub Issues反馈。

---

**作者**: AI Code Assistant
**版本**: 1.0
**日期**: 2026-02-10
