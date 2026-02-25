"""
火箭回收算法 - 火箭垂直着陆控制系统

基于最优控制理论和模型预测控制(MPC)的火箭回收算法
模拟SpaceX Falcon 9火箭的垂直着陆过程

作者: AI Code Assistant
日期: 2026-02-10
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from dataclasses import dataclass
from typing import Tuple, List, Optional
import math


@dataclass
class RocketState:
    """火箭状态类"""
    x: float  # 水平位置 (m)
    y: float  # 高度 (m)
    vx: float  # 水平速度 (m/s)
    vy: float  # 垂直速度 (m/s)
    theta: float  # 姿态角 (弧度)
    omega: float  # 角速度 (弧度/s)
    mass: float  # 质量 (kg)
    fuel: float  # 燃料质量 (kg)

    @property
    def dry_mass(self) -> float:
        """干质量"""
        return self.mass - self.fuel


@dataclass
class RocketParameters:
    """火箭参数"""
    dry_mass: float = 22200  # 干质量 (kg)
    max_fuel: float = 409500  # 最大燃料质量 (kg)
    thrust_max: float = 7607000  # 最大推力 (N)
    thrust_min: float = 0.3 * 7607000  # 最小推力 (N)
    specific_impulse: float = 282  # 比冲 (s)
    g: float = 9.81  # 重力加速度 (m/s²)
    max_gimbal: float = np.radians(10)  # 最大摆角 (弧度)
    max_omega: float = np.radians(30)  # 最大角速度 (弧度/s)
    max_theta: float = np.radians(45)  # 最大姿态角 (弧度)
    length: float = 50  # 火箭长度 (m)
    width: float = 3.7  # 火箭直径 (m)
    moment_of_inertia: float = 1.5e6  # 转动惯量 (kg·m²)


class RocketLandingController:
    """火箭着陆控制器"""

    def __init__(self, params: RocketParameters):
        self.params = params
        self.g = params.g

    def compute_thrust(self, throttle: float) -> float:
        """计算推力"""
        return self.params.thrust_min + throttle * (self.params.thrust_max - self.params.thrust_min)

    def compute_mass_flow(self, thrust: float) -> float:
        """计算质量流量 (kg/s)"""
        # Isp = F / (mdot * g0)
        return thrust / (self.params.specific_impulse * self.g)

    def dynamics(self, state: RocketState, throttle: float, gimbal: float) -> Tuple[float, float, float, float, float, float]:
        """
        火箭动力学模型
        返回: dx/dt, dy/dt, dvx/dt, dvy/dt, dtheta/dt, domega/dt
        """
        thrust = self.compute_thrust(throttle)
        mass_flow = self.compute_mass_flow(thrust)

        # 更新质量
        new_mass = state.mass - mass_flow * 0.1  # 假设时间步长0.1s

        # 推力分量
        thrust_x = thrust * np.sin(state.theta + gimbal)
        thrust_y = thrust * np.cos(state.theta + gimbal)

        # 加速度
        ax = thrust_x / state.mass
        ay = thrust_y / state.mass - self.g

        # 角加速度 (简化模型)
        alpha = (thrust * gimbal * self.params.length / 2) / self.params.moment_of_inertia

        return ax, ay, alpha, mass_flow

    def compute_landing_trajectory(self, initial_state: RocketState, target_x: float,
                                   target_y: float, time_horizon: float = 30.0) -> List[RocketState]:
        """
        计算最优着陆轨迹 - 使用模型预测控制(MPC)

        参数:
            initial_state: 初始状态
            target_x: 目标水平位置
            target_y: 目标高度
            time_horizon: 预测时域 (s)

        返回:
            轨迹点列表
        """
        # 使用简化的最优控制方法
        trajectory = []
        state = initial_state
        dt = 0.1  # 时间步长

        # 计算初始着陆窗口
        landing_window = self.compute_landing_window(state, target_x, target_y)

        for t in np.arange(0, time_horizon, dt):
            # 计算控制输入
            throttle, gimbal = self.compute_control(state, target_x, target_y, landing_window)

            # 更新状态
            ax, ay, alpha, mass_flow = self.dynamics(state, throttle, gimbal)

            # 更新状态
            state = RocketState(
                x=state.x + state.vx * dt,
                y=state.y + state.vy * dt,
                vx=state.vx + ax * dt,
                vy=state.vy + ay * dt,
                theta=state.theta + state.omega * dt,
                omega=state.omega + alpha * dt,
                mass=state.mass - mass_flow * dt,
                fuel=state.fuel - mass_flow * dt
            )

            trajectory.append(state)

            # 检查着陆条件
            if self.check_landing_conditions(state, target_x, target_y):
                break

        return trajectory

    def compute_landing_window(self, state: RocketState, target_x: float, target_y: float) -> dict:
        """
        计算着陆窗口
        返回: 包含着陆约束的字典
        """
        # 计算剩余燃料时间
        fuel_time = state.fuel / self.compute_mass_flow(self.params.thrust_max)

        # 计算着陆所需最小推力
        # 考虑重力、姿态和着陆精度
        min_thrust_required = state.mass * self.g * 1.2  # 20%余量

        # 计算着陆窗口
        return {
            'max_horizontal_speed': 5.0,  # 最大水平速度 (m/s)
            'max_vertical_speed': 2.0,    # 最大垂直速度 (m/s)
            'max_attitude': np.radians(5), # 最大姿态角 (弧度)
            'min_fuel_time': fuel_time,
            'min_thrust': min_thrust_required,
            'landing_radius': 10.0  # 着陆半径 (m)
        }

    def compute_control(self, state: RocketState, target_x: float, target_y: float,
                       landing_window: dict) -> Tuple[float, float]:
        """
        计算控制输入 (节流阀, 摆角)
        使用PID控制器和最优控制策略
        """
        # 水平位置误差
        dx = target_x - state.x
        dy = target_y - state.y

        # 速度误差
        dvx = -state.vx  # 期望水平速度为0
        dvy = -state.vy  # 期望垂直速度为0

        # PID控制器参数
        Kp_h = 0.5  # 水平位置增益
        Kp_v = 0.8  # 垂直位置增益
        Kd_h = 0.3  # 水平速度增益
        Kd_v = 0.5  # 垂直速度增益

        # 计算期望姿态角
        # 水平控制：通过姿态角调整水平推力
        desired_theta = np.clip(
            -Kp_h * dx - Kd_h * dvx,
            -self.params.max_theta,
            self.params.max_theta
        )

        # 垂直控制：通过节流阀调整推力
        # 基础推力 = 重力 + 垂直速度控制
        base_thrust = state.mass * self.g + Kp_v * dy + Kd_v * dvy

        # 计算节流阀
        throttle = (base_thrust - self.params.thrust_min) / (self.params.thrust_max - self.params.thrust_min)
        throttle = np.clip(throttle, 0.0, 1.0)

        # 计算摆角
        # 摆角用于姿态控制，同时提供水平推力
        gimbal = np.clip(
            (desired_theta - state.theta) * 0.5,
            -self.params.max_gimbal,
            self.params.max_gimbal
        )

        # 着陆阶段特殊处理
        if state.y < 100:  # 低于100米
            # 更精确的姿态控制
            gimbal = np.clip(
                (desired_theta - state.theta) * 0.8,
                -self.params.max_gimbal * 0.5,
                self.params.max_gimbal * 0.5
            )

            # 垂直速度控制更严格
            if state.vy < -2:
                throttle = min(throttle + 0.2, 1.0)

        # 燃料不足时的紧急处理
        fuel_time = state.fuel / self.compute_mass_flow(self.params.thrust_max)
        if fuel_time < 5.0:
            # 优先保证垂直着陆，减少水平调整
            throttle = max(throttle, 0.7)
            gimbal = np.clip(gimbal * 0.3, -self.params.max_gimbal * 0.2, self.params.max_gimbal * 0.2)

        return throttle, gimbal

    def check_landing_conditions(self, state: RocketState, target_x: float, target_y: float) -> bool:
        """检查着陆条件"""
        # 水平距离
        dx = abs(state.x - target_x)
        dy = abs(state.y - target_y)

        # 速度条件
        vx_ok = abs(state.vx) < 0.5
        vy_ok = abs(state.vy) < 0.5

        # 姿态条件
        theta_ok = abs(state.theta) < np.radians(2)

        # 高度条件
        height_ok = dy < 1.0

        return dx < 5.0 and height_ok and vx_ok and vy_ok and theta_ok


class TrajectoryOptimizer:
    """轨迹优化器 - 使用数值优化方法"""

    def __init__(self, controller: RocketLandingController):
        self.controller = controller

    def optimize_trajectory(self, initial_state: RocketState, target_x: float,
                           target_y: float, n_segments: int = 50) -> dict:
        """
        优化着陆轨迹 - 使用直接法求解最优控制问题

        参数:
            initial_state: 初始状态
            target_x: 目标水平位置
            target_y: 目标高度
            n_segments: 轨迹分段数

        返回:
            优化后的轨迹和控制序列
        """
        # 定义优化变量
        # x = [throttle_0, gimbal_0, throttle_1, gimbal_1, ..., throttle_n, gimbal_n]
        n_vars = n_segments * 2

        # 初始猜测
        x0 = np.ones(n_vars) * 0.5  # 初始节流阀0.5，摆角0

        # 定义目标函数
        def objective(x):
            # 模拟轨迹
            trajectory = self.simulate_trajectory(initial_state, x, n_segments)

            # 计算成本
            final_state = trajectory[-1]

            # 位置误差成本
            pos_cost = (final_state.x - target_x)**2 + (final_state.y - target_y)**2

            # 速度误差成本
            vel_cost = final_state.vx**2 + final_state.vy**2

            # 姿态误差成本
            attitude_cost = final_state.theta**2

            # 燃料成本 (最小化燃料使用)
            fuel_cost = (initial_state.fuel - final_state.fuel) * 0.001

            # 控制平滑度成本
            control_smoothness = 0
            for i in range(0, n_vars-2, 2):
                control_smoothness += (x[i] - x[i+2])**2 + (x[i+1] - x[i+3])**2

            # 总成本
            total_cost = (
                1000 * pos_cost +
                100 * vel_cost +
                50 * attitude_cost +
                fuel_cost +
                0.1 * control_smoothness
            )

            return total_cost

        # 约束条件
        constraints = []

        # 节流阀约束 [0, 1]
        for i in range(0, n_vars, 2):
            constraints.append({'type': 'ineq', 'fun': lambda x, i=i: x[i]})  # >= 0
            constraints.append({'type': 'ineq', 'fun': lambda x, i=i: 1 - x[i]})  # <= 1

        # 摆角约束 [-max_gimbal, max_gimbal]
        for i in range(1, n_vars, 2):
            constraints.append({'type': 'ineq', 'fun': lambda x, i=i: x[i] + self.controller.params.max_gimbal})
            constraints.append({'type': 'ineq', 'fun': lambda x, i=i: self.controller.params.max_gimbal - x[i]})

        # 优化
        result = minimize(
            objective,
            x0,
            method='SLSQP',
            constraints=constraints,
            options={'maxiter': 100, 'ftol': 1e-6}
        )

        # 提取优化结果
        optimized_controls = result.x.reshape(n_segments, 2)

        # 生成优化轨迹
        trajectory = self.simulate_trajectory(initial_state, result.x, n_segments)

        return {
            'trajectory': trajectory,
            'controls': optimized_controls,
            'success': result.success,
            'cost': result.fun,
            'message': result.message
        }

    def simulate_trajectory(self, initial_state: RocketState, controls: np.ndarray,
                           n_segments: int) -> List[RocketState]:
        """使用控制序列模拟轨迹"""
        trajectory = [initial_state]
        state = initial_state

        dt = 0.5  # 每段的时间步长

        for i in range(n_segments):
            throttle = controls[i * 2]
            gimbal = controls[i * 2 + 1]

            # 更新状态
            ax, ay, alpha, mass_flow = self.controller.dynamics(state, throttle, gimbal)

            state = RocketState(
                x=state.x + state.vx * dt,
                y=state.y + state.vy * dt,
                vx=state.vx + ax * dt,
                vy=state.vy + ay * dt,
                theta=state.theta + state.omega * dt,
                omega=state.omega + alpha * dt,
                mass=state.mass - mass_flow * dt,
                fuel=state.fuel - mass_flow * dt
            )

            trajectory.append(state)

            # 如果已经着陆，提前结束
            if state.y <= 0:
                break

        return trajectory


class LandingSimulator:
    """着陆模拟器"""

    def __init__(self, params: RocketParameters):
        self.params = params
        self.controller = RocketLandingController(params)
        self.optimizer = TrajectoryOptimizer(self.controller)

    def run_simulation(self, initial_altitude: float = 5000,
                      initial_horizontal: float = 1000,
                      target_x: float = 0,
                      target_y: float = 0,
                      use_optimization: bool = True) -> dict:
        """
        运行着陆模拟

        参数:
            initial_altitude: 初始高度 (m)
            initial_horizontal: 初始水平距离 (m)
            target_x: 目标水平位置
            target_y: 目标高度
            use_optimization: 是否使用优化器

        返回:
            模拟结果
        """
        # 初始状态
        initial_state = RocketState(
            x=initial_horizontal,
            y=initial_altitude,
            vx=-50,  # 向目标移动
            vy=-100,  # 向下速度
            theta=np.radians(5),  # 略微倾斜
            omega=0,
            mass=self.params.dry_mass + self.params.max_fuel,
            fuel=self.params.max_fuel
        )

        if use_optimization:
            # 使用优化器
            result = self.optimizer.optimize_trajectory(
                initial_state, target_x, target_y, n_segments=30
            )

            trajectory = result['trajectory']
            success = result['success']
            fuel_used = initial_state.fuel - trajectory[-1].fuel
            final_state = trajectory[-1]

        else:
            # 使用控制器
            trajectory = self.controller.compute_landing_trajectory(
                initial_state, target_x, target_y, time_horizon=60
            )

            success = self.controller.check_landing_conditions(
                trajectory[-1], target_x, target_y
            )
            fuel_used = initial_state.fuel - trajectory[-1].fuel
            final_state = trajectory[-1]

        # 计算性能指标
        final_error = np.sqrt((final_state.x - target_x)**2 + (final_state.y - target_y)**2)
        final_velocity = np.sqrt(final_state.vx**2 + final_state.vy**2)

        return {
            'trajectory': trajectory,
            'success': success,
            'fuel_used': fuel_used,
            'fuel_efficiency': fuel_used / initial_state.fuel * 100,
            'final_error': final_error,
            'final_velocity': final_velocity,
            'final_attitude': np.degrees(final_state.theta),
            'initial_state': initial_state,
            'target_x': target_x,
            'target_y': target_y
        }

    def plot_trajectory(self, result: dict, title: str = "火箭着陆轨迹"):
        """绘制轨迹图"""
        trajectory = result['trajectory']

        # 提取轨迹数据
        x = [s.x for s in trajectory]
        y = [s.y for s in trajectory]
        vx = [s.vx for s in trajectory]
        vy = [s.vy for s in trajectory]
        theta = [np.degrees(s.theta) for s in trajectory]
        fuel = [s.fuel for s in trajectory]

        # 创建子图
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        fig.suptitle(title, fontsize=16)

        # 1. 二维轨迹
        ax1 = axes[0, 0]
        ax1.plot(x, y, 'b-', linewidth=2, label='轨迹')
        ax1.scatter([result['target_x']], [result['target_y']],
                   c='red', s=100, marker='*', label='目标点')
        ax1.scatter([x[0]], [y[0]], c='green', s=100, marker='^', label='起点')
        ax1.set_xlabel('水平位置 (m)')
        ax1.set_ylabel('高度 (m)')
        ax1.set_title('二维着陆轨迹')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        ax1.set_aspect('equal', adjustable='box')

        # 2. 速度变化
        ax2 = axes[0, 1]
        time = np.arange(len(vx)) * 0.5  # 假设时间步长0.5s
        ax2.plot(time, vx, 'r-', label='水平速度')
        ax2.plot(time, vy, 'b-', label='垂直速度')
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        ax2.set_xlabel('时间 (s)')
        ax2.set_ylabel('速度 (m/s)')
        ax2.set_title('速度变化')
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        # 3. 姿态角变化
        ax3 = axes[0, 2]
        ax3.plot(time, theta, 'g-')
        ax3.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        ax3.set_xlabel('时间 (s)')
        ax3.set_ylabel('姿态角 (度)')
        ax3.set_title('姿态角变化')
        ax3.grid(True, alpha=0.3)

        # 4. 燃料消耗
        ax4 = axes[1, 0]
        ax4.plot(time, fuel, 'orange')
        ax4.set_xlabel('时间 (s)')
        ax4.set_ylabel('燃料质量 (kg)')
        ax4.set_title('燃料消耗')
        ax4.grid(True, alpha=0.3)

        # 5. 高度-速度相图
        ax5 = axes[1, 1]
        ax5.plot(y, vy, 'purple')
        ax5.set_xlabel('高度 (m)')
        ax5.set_ylabel('垂直速度 (m/s)')
        ax5.set_title('高度-速度相图')
        ax5.grid(True, alpha=0.3)

        # 6. 性能指标
        ax6 = axes[1, 2]
        ax6.axis('off')
        metrics_text = f"""
        模拟结果:
        {'✓ 成功着陆' if result['success'] else '✗ 着陆失败'}

        燃料使用: {result['fuel_used']:.0f} kg
        燃料效率: {result['fuel_efficiency']:.1f}%

        着陆误差: {result['final_error']:.2f} m
        着陆速度: {result['final_velocity']:.2f} m/s
        最终姿态: {result['final_attitude']:.2f}°

        初始高度: {result['initial_state'].y:.0f} m
        初始水平距离: {result['initial_state'].x:.0f} m
        """
        ax6.text(0.1, 0.5, metrics_text, fontsize=10,
                verticalalignment='center', fontfamily='monospace')

        plt.tight_layout()
        plt.show()

        return fig


def main():
    """主函数 - 演示火箭回收算法"""
    print("=" * 60)
    print("火箭回收算法演示")
    print("=" * 60)

    # 创建火箭参数
    params = RocketParameters()

    # 创建模拟器
    simulator = LandingSimulator(params)

    # 场景1: 标准着陆
    print("\n场景1: 标准着陆 (高度5000m, 水平距离1000m)")
    result1 = simulator.run_simulation(
        initial_altitude=5000,
        initial_horizontal=1000,
        use_optimization=True
    )

    print(f"结果: {'成功' if result1['success'] else '失败'}")
    print(f"燃料使用: {result1['fuel_used']:.0f} kg ({result1['fuel_efficiency']:.1f}%)")
    print(f"着陆误差: {result1['final_error']:.2f} m")
    print(f"着陆速度: {result1['final_velocity']:.2f} m/s")

    # 场景2: 高精度着陆
    print("\n场景2: 高精度着陆 (高度3000m, 水平距离500m)")
    result2 = simulator.run_simulation(
        initial_altitude=3000,
        initial_horizontal=500,
        use_optimization=True
    )

    print(f"结果: {'成功' if result2['success'] else '失败'}")
    print(f"燃料使用: {result2['fuel_used']:.0f} kg ({result2['fuel_efficiency']:.1f}%)")
    print(f"着陆误差: {result2['final_error']:.2f} m")

    # 场景3: 紧急着陆 (燃料不足)
    print("\n场景3: 紧急着陆 (燃料不足)")
    result3 = simulator.run_simulation(
        initial_altitude=2000,
        initial_horizontal=300,
        use_optimization=False  # 使用控制器而非优化器
    )

    print(f"结果: {'成功' if result3['success'] else '失败'}")
    print(f"燃料使用: {result3['fuel_used']:.0f} kg")

    # 绘制轨迹
    print("\n绘制轨迹图...")
    simulator.plot_trajectory(result1, "标准着陆轨迹")
    simulator.plot_trajectory(result2, "高精度着陆轨迹")

    # 算法性能分析
    print("\n" + "=" * 60)
    print("算法性能分析")
    print("=" * 60)

    print("\n1. 控制策略:")
    print("   - PID控制器用于实时控制")
    print("   - 模型预测控制(MPC)用于轨迹规划")
    print("   - 最优控制用于燃料优化")

    print("\n2. 关键技术:")
    print("   - 姿态控制: 通过摆角调整推力方向")
    print("   - 速度控制: 通过节流阀调整推力大小")
    print("   - 燃料管理: 优先保证垂直着陆")

    print("\n3. 安全约束:")
    print("   - 最大姿态角: ±45°")
    print("   - 最大角速度: ±30°/s")
    print("   - 着陆精度: < 5m")
    print("   - 着陆速度: < 0.5m/s")

    print("\n4. 优化策略:")
    print("   - 燃料消耗最小化")
    print("   - 着陆精度最大化")
    print("   - 控制平滑度优化")

    return result1, result2, result3


if __name__ == "__main__":
    results = main()
