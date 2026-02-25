"""
火箭回收算法 - 简化版本

用于快速验证火箭着陆控制算法的核心逻辑
"""

import numpy as np
import matplotlib.pyplot as plt


class SimpleRocket:
    """简化火箭模型"""

    def __init__(self):
        # 物理参数
        self.mass = 25000  # 质量 (kg)
        self.fuel = 400000  # 燃料 (kg)
        self.thrust_max = 7607000  # 最大推力 (N)
        self.g = 9.81  # 重力加速度 (m/s²)
        self.Isp = 282  # 比冲 (s)

        # 状态
        self.x = 0  # 水平位置 (m)
        self.y = 5000  # 高度 (m)
        self.vx = -50  # 水平速度 (m/s)
        self.vy = -100  # 垂直速度 (m/s)
        self.theta = 0  # 姿态角 (弧度)
        self.omega = 0  # 角速度 (弧度/s)

        # 目标
        self.target_x = 0
        self.target_y = 0

    def update(self, throttle: float, gimbal: float, dt: float = 0.1):
        """更新火箭状态"""
        # 计算推力
        thrust = throttle * self.thrust_max

        # 计算质量流量
        mdot = thrust / (self.Isp * self.g)

        # 更新质量
        if self.fuel > 0:
            self.fuel -= mdot * dt
            self.mass = 25000 + self.fuel
        else:
            thrust = 0  # 燃料耗尽

        # 推力分量
        thrust_x = thrust * np.sin(self.theta + gimbal)
        thrust_y = thrust * np.cos(self.theta + gimbal)

        # 加速度
        ax = thrust_x / self.mass
        ay = thrust_y / self.mass - self.g

        # 角加速度 (简化)
        alpha = thrust * gimbal * 25 / 1000000  # 简化转动惯量

        # 更新状态
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.vx += ax * dt
        self.vy += ay * dt
        self.theta += self.omega * dt
        self.omega += alpha * dt

        # 限制姿态
        self.theta = np.clip(self.theta, -np.radians(45), np.radians(45))

        return thrust, ax, ay

    def get_state(self):
        """获取当前状态"""
        return {
            'x': self.x,
            'y': self.y,
            'vx': self.vx,
            'vy': self.vy,
            'theta': np.degrees(self.theta),
            'omega': np.degrees(self.omega),
            'mass': self.mass,
            'fuel': self.fuel
        }


class LandingController:
    """着陆控制器"""

    def __init__(self):
        # PID参数
        self.Kp_x = 0.02  # 水平位置增益
        self.Kp_y = 0.05  # 垂直位置增益
        self.Kd_x = 0.01  # 水平速度增益
        self.Kd_y = 0.03  # 垂直速度增益

        # 控制历史
        self.history = []

    def compute_control(self, rocket: SimpleRocket) -> tuple:
        """计算控制输入 (节流阀, 摆角)"""
        # 位置误差
        dx = rocket.target_x - rocket.x
        dy = rocket.target_y - rocket.y

        # 速度误差
        dvx = -rocket.vx  # 期望水平速度为0
        dvy = -rocket.vy  # 期望垂直速度为0

        # 计算期望姿态角
        desired_theta = -self.Kp_x * dx - self.Kd_x * dvx

        # 计算节流阀
        # 基础推力 = 重力 + 垂直速度控制
        base_thrust = rocket.mass * rocket.g + self.Kp_y * dy + self.Kd_y * dvy

        # 转换为节流阀
        throttle = base_thrust / rocket.thrust_max
        throttle = np.clip(throttle, 0.0, 1.0)

        # 计算摆角
        gimbal = (desired_theta - rocket.theta) * 0.5
        gimbal = np.clip(gimbal, -np.radians(10), np.radians(10))

        # 着陆阶段优化
        if rocket.y < 100:
            # 降低摆角，提高稳定性
            gimbal *= 0.3
            # 增加垂直推力
            if rocket.vy < -2:
                throttle = min(throttle + 0.2, 1.0)

        # 燃料不足时的紧急处理
        if rocket.fuel < 50000:
            throttle = max(throttle, 0.7)
            gimbal *= 0.2

        # 记录控制历史
        self.history.append({
            'throttle': throttle,
            'gimbal': np.degrees(gimbal),
            'desired_theta': np.degrees(desired_theta),
            'actual_theta': np.degrees(rocket.theta)
        })

        return throttle, gimbal

    def check_landing(self, rocket: SimpleRocket) -> bool:
        """检查是否成功着陆"""
        dx = abs(rocket.x - rocket.target_x)
        dy = abs(rocket.y - rocket.target_y)

        vx_ok = abs(rocket.vx) < 0.5
        vy_ok = abs(rocket.vy) < 0.5
        theta_ok = abs(rocket.theta) < np.radians(2)
        height_ok = dy < 1.0

        return dx < 5.0 and height_ok and vx_ok and vy_ok and theta_ok


def simulate_landing(initial_altitude=5000, initial_horizontal=1000, max_time=60):
    """模拟着陆过程"""
    rocket = SimpleRocket()
    controller = LandingController()

    # 设置初始状态
    rocket.y = initial_altitude
    rocket.x = initial_horizontal
    rocket.vx = -50  # 向目标移动
    rocket.vy = -100  # 向下速度

    # 记录轨迹
    trajectory = []
    dt = 0.1

    print(f"初始状态: 高度={rocket.y}m, 水平距离={rocket.x}m")
    print(f"初始速度: vx={rocket.vx}m/s, vy={rocket.vy}m/s")
    print("开始着陆模拟...")

    for t in range(int(max_time / dt)):
        # 获取控制输入
        throttle, gimbal = controller.compute_control(rocket)

        # 更新火箭状态
        thrust, ax, ay = rocket.update(throttle, gimbal, dt)

        # 记录状态
        state = rocket.get_state()
        state['time'] = t * dt
        state['throttle'] = throttle
        state['gimbal'] = np.degrees(gimbal)
        state['thrust'] = thrust
        trajectory.append(state)

        # 检查着陆
        if controller.check_landing(rocket):
            print(f"成功着陆! 时间: {t*dt:.1f}s")
            return trajectory, True

        # 检查坠毁
        if rocket.y <= 0:
            print(f"坠毁! 时间: {t*dt:.1f}s")
            return trajectory, False

        # 打印进度
        if t % 50 == 0:
            print(f"t={t*dt:.1f}s: 高度={rocket.y:.0f}m, 速度={rocket.vy:.1f}m/s, 燃料={rocket.fuel:.0f}kg")

    print("模拟超时")
    return trajectory, False


def plot_results(trajectory, success):
    """绘制结果图"""
    if not trajectory:
        return

    # 提取数据
    times = [s['time'] for s in trajectory]
    heights = [s['y'] for s in trajectory]
    horizontal = [s['x'] for s in trajectory]
    vx = [s['vx'] for s in trajectory]
    vy = [s['vy'] for s in trajectory]
    theta = [s['theta'] for s in trajectory]
    throttle = [s['throttle'] for s in trajectory]
    fuel = [s['fuel'] for s in trajectory]

    # 创建图形
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    fig.suptitle('火箭着陆模拟结果', fontsize=16)

    # 1. 二维轨迹
    ax1 = axes[0, 0]
    ax1.plot(horizontal, heights, 'b-', linewidth=2)
    ax1.scatter([0], [0], c='red', s=100, marker='*', label='目标点')
    ax1.scatter([horizontal[0]], [heights[0]], c='green', s=100, marker='^', label='起点')
    ax1.set_xlabel('水平位置 (m)')
    ax1.set_ylabel('高度 (m)')
    ax1.set_title('着陆轨迹')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_aspect('equal', adjustable='box')

    # 2. 高度变化
    ax2 = axes[0, 1]
    ax2.plot(times, heights, 'b-')
    ax2.axhline(y=0, color='r', linestyle='--', alpha=0.5)
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('高度 (m)')
    ax2.set_title('高度变化')
    ax2.grid(True, alpha=0.3)

    # 3. 速度变化
    ax3 = axes[0, 2]
    ax3.plot(times, vx, 'r-', label='水平速度')
    ax3.plot(times, vy, 'b-', label='垂直速度')
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax3.set_xlabel('时间 (s)')
    ax3.set_ylabel('速度 (m/s)')
    ax3.set_title('速度变化')
    ax3.grid(True, alpha=0.3)
    ax3.legend()

    # 4. 姿态角变化
    ax4 = axes[1, 0]
    ax4.plot(times, theta, 'g-')
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax4.set_xlabel('时间 (s)')
    ax4.set_ylabel('姿态角 (度)')
    ax4.set_title('姿态角变化')
    ax4.grid(True, alpha=0.3)

    # 5. 节流阀控制
    ax5 = axes[1, 1]
    ax5.plot(times, throttle, 'orange')
    ax5.set_xlabel('时间 (s)')
    ax5.set_ylabel('节流阀')
    ax5.set_title('节流阀控制')
    ax5.grid(True, alpha=0.3)
    ax5.set_ylim(0, 1.1)

    # 6. 燃料消耗
    ax6 = axes[1, 2]
    ax6.plot(times, fuel, 'purple')
    ax6.set_xlabel('时间 (s)')
    ax6.set_ylabel('燃料 (kg)')
    ax6.set_title('燃料消耗')
    ax6.grid(True, alpha=0.3)

    # 7. 高度-速度相图
    ax7 = axes[2, 0]
    ax7.plot(heights, vy, 'cyan')
    ax7.set_xlabel('高度 (m)')
    ax7.set_ylabel('垂直速度 (m/s)')
    ax7.set_title('高度-速度相图')
    ax7.grid(True, alpha=0.3)

    # 8. 控制历史
    ax8 = axes[2, 1]
    controller = LandingController()
    if controller.history:
        gimbal_history = [h['gimbal'] for h in controller.history]
        desired_theta = [h['desired_theta'] for h in controller.history]
        actual_theta = [h['actual_theta'] for h in controller.history]

        ax8.plot(times[:len(gimbal_history)], gimbal_history, 'r-', label='摆角')
        ax8.plot(times[:len(desired_theta)], desired_theta, 'b--', label='期望姿态')
        ax8.plot(times[:len(actual_theta)], actual_theta, 'g:', label='实际姿态')
        ax8.set_xlabel('时间 (s)')
        ax8.set_ylabel('角度 (度)')
        ax8.set_title('控制历史')
        ax8.grid(True, alpha=0.3)
        ax8.legend()

    # 9. 性能指标
    ax9 = axes[2, 2]
    ax9.axis('off')

    if trajectory:
        final_state = trajectory[-1]
        fuel_used = 400000 - final_state['fuel']
        final_error = np.sqrt(final_state['x']**2 + final_state['y']**2)
        final_velocity = np.sqrt(final_state['vx']**2 + final_state['vy']**2)

        metrics_text = f"""
        模拟结果:
        {'✓ 成功着陆' if success else '✗ 着陆失败'}

        燃料使用: {fuel_used:.0f} kg
        燃料效率: {fuel_used/400000*100:.1f}%

        着陆误差: {final_error:.2f} m
        着陆速度: {final_velocity:.2f} m/s
        最终姿态: {final_state['theta']:.2f}°

        初始高度: {trajectory[0]['y']:.0f} m
        初始水平距离: {trajectory[0]['x']:.0f} m
        """
        ax9.text(0.1, 0.5, metrics_text, fontsize=10,
                verticalalignment='center', fontfamily='monospace')

    plt.tight_layout()
    plt.show()


def main():
    """主函数"""
    print("=" * 60)
    print("火箭回收算法 - 简化版本")
    print("=" * 60)

    # 场景1: 标准着陆
    print("\n场景1: 标准着陆 (高度5000m, 水平距离1000m)")
    trajectory1, success1 = simulate_landing(
        initial_altitude=5000,
        initial_horizontal=1000,
        max_time=60
    )

    # 场景2: 高精度着陆
    print("\n场景2: 高精度着陆 (高度3000m, 水平距离500m)")
    trajectory2, success2 = simulate_landing(
        initial_altitude=3000,
        initial_horizontal=500,
        max_time=40
    )

    # 绘制结果
    print("\n绘制结果图...")
    plot_results(trajectory1, success1)
    plot_results(trajectory2, success2)

    # 算法总结
    print("\n" + "=" * 60)
    print("算法总结")
    print("=" * 60)
    print("\n1. 控制策略:")
    print("   - PID控制器实时调整姿态和推力")
    print("   - 水平控制: 通过姿态角调整水平推力")
    print("   - 垂直控制: 通过节流阀调整推力大小")
    print("   - 着陆阶段: 降低摆角，提高稳定性")

    print("\n2. 优化措施:")
    print("   - 燃料不足时优先保证垂直着陆")
    print("   - 低高度时降低控制增益，避免振荡")
    print("   - 实时监控着陆条件，及时调整策略")

    print("\n3. 性能指标:")
    print("   - 着陆精度: < 5m")
    print("   - 着陆速度: < 0.5m/s")
    print("   - 姿态误差: < 2°")
    print("   - 燃料效率: 优化控制减少燃料消耗")


if __name__ == "__main__":
    main()
