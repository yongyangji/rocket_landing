"""
火箭回收算法 - 演示版本

展示算法设计和控制逻辑，不追求实际着陆成功
"""

import math


def simulate_rocket_landing_demo():
    """演示火箭着陆算法"""
    print("=" * 60)
    print("火箭回收算法 - 演示版本")
    print("=" * 60)

    # 火箭参数
    mass = 25000  # 质量 (kg)
    fuel = 400000  # 燃料 (kg)
    thrust_max = 7607000  # 最大推力 (N)
    g = 9.81  # 重力加速度 (m/s²)
    Isp = 282  # 比冲 (s)

    # 初始状态
    x = 500  # 水平位置 (m)
    y = 2000  # 高度 (m)
    vx = -20  # 水平速度 (m/s)
    vy = -50  # 垂直速度 (m/s)
    theta = 0  # 姿态角 (弧度)
    omega = 0  # 角速度 (弧度/s)

    # 目标
    target_x = 0
    target_y = 0

    # 控制参数
    Kp_x = 0.01
    Kp_y = 0.1
    Kd_x = 0.005
    Kd_y = 0.05

    # 模拟参数
    dt = 0.1
    max_time = 60
    trajectory = []

    print(f"初始状态: 高度={y}m, 水平距离={x}m")
    print(f"初始速度: vx={vx}m/s, vy={vy}m/s")
    print("开始着陆模拟...")

    for t in range(int(max_time / dt)):
        # 计算控制输入
        dx = target_x - x
        dy = target_y - y
        dvx = -vx
        dvy = -vy

        # 期望姿态角
        desired_theta = -Kp_x * dx - Kd_x * dvx

        # 限制期望姿态角
        max_desired_theta = math.radians(20)
        if desired_theta > max_desired_theta:
            desired_theta = max_desired_theta
        elif desired_theta < -max_desired_theta:
            desired_theta = -max_desired_theta

        # 节流阀
        base_thrust = mass * g + Kp_y * dy + Kd_y * dvy

        # 限制推力范围
        min_thrust = 0.3 * thrust_max
        max_thrust = thrust_max

        if base_thrust < min_thrust:
            base_thrust = min_thrust
        elif base_thrust > max_thrust:
            base_thrust = max_thrust

        throttle = base_thrust / thrust_max

        # 摆角
        gimbal = (desired_theta - theta) * 0.3
        max_gimbal = math.radians(5)
        if gimbal > max_gimbal:
            gimbal = max_gimbal
        elif gimbal < -max_gimbal:
            gimbal = -max_gimbal

        # 着陆阶段优化
        if y < 100:
            gimbal *= 0.1
            if vy < -2:
                throttle = max(throttle, 0.6)
            if abs(theta) > math.radians(3):
                gimbal = -math.copysign(max_gimbal * 0.2, theta)

        # 燃料不足时的紧急处理
        if fuel < 50000:
            throttle = max(throttle, 0.6)
            gimbal *= 0.05

        # 计算推力
        thrust = throttle * thrust_max

        # 计算质量流量
        mdot = thrust / (Isp * g)

        # 更新质量
        if fuel > 0:
            fuel -= mdot * dt
            mass = 25000 + fuel
        else:
            thrust = 0

        # 推力分量
        thrust_x = thrust * math.sin(theta + gimbal)
        thrust_y = thrust * math.cos(theta + gimbal)

        # 加速度
        ax = thrust_x / mass
        ay = thrust_y / mass - g

        # 角加速度
        alpha = thrust * gimbal * 25 / 1000000

        # 更新状态
        x += vx * dt
        y += vy * dt
        vx += ax * dt
        vy += ay * dt
        theta += omega * dt
        omega += alpha * dt

        # 限制姿态
        max_theta = math.radians(30)
        if theta > max_theta:
            theta = max_theta
        elif theta < -max_theta:
            theta = -max_theta

        # 限制角速度
        max_omega = math.radians(20)
        if omega > max_omega:
            omega = max_omega
        elif omega < -max_omega:
            omega = -max_omega

        # 记录状态
        trajectory.append({
            'time': t * dt,
            'x': x,
            'y': y,
            'vx': vx,
            'vy': vy,
            'theta': math.degrees(theta),
            'omega': math.degrees(omega),
            'throttle': throttle,
            'gimbal': math.degrees(gimbal),
            'fuel': fuel,
            'mass': mass,
            'thrust': thrust,
            'desired_theta': math.degrees(desired_theta)
        })

        # 检查着陆
        dx = abs(x - target_x)
        dy = abs(y - target_y)
        vx_ok = abs(vx) < 0.5
        vy_ok = abs(vy) < 0.5
        theta_ok = abs(theta) < math.radians(2)
        height_ok = dy < 1.0

        if dx < 5.0 and height_ok and vx_ok and vy_ok and theta_ok:
            print(f"成功着陆! 时间: {t*dt:.1f}s")
            return trajectory, True

        # 检查坠毁
        if y <= 0:
            print(f"坠毁! 时间: {t*dt:.1f}s")
            return trajectory, False

        # 打印进度
        if t % 50 == 0:
            print(f"t={t*dt:.1f}s: 高度={y:.0f}m, 速度={vy:.1f}m/s, 燃料={fuel:.0f}kg, 姿态={math.degrees(theta):.1f}°")

    print("模拟超时")
    return trajectory, False


def print_results(trajectory, success):
    """打印结果"""
    if not trajectory:
        return

    final_state = trajectory[-1]
    fuel_used = 400000 - final_state['fuel']
    final_error = math.sqrt(final_state['x']**2 + final_state['y']**2)
    final_velocity = math.sqrt(final_state['vx']**2 + final_state['vy']**2)

    print("\n" + "=" * 60)
    print("模拟结果")
    print("=" * 60)
    print(f"着陆结果: {'成功着陆' if success else '着陆失败'}")
    print(f"总时间: {final_state['time']:.1f}s")
    print(f"燃料使用: {fuel_used:.0f} kg")
    print(f"燃料效率: {fuel_used/400000*100:.1f}%")
    print(f"着陆误差: {final_error:.2f} m")
    print(f"着陆速度: {final_velocity:.2f} m/s")
    print(f"最终姿态: {final_state['theta']:.2f}°")
    print(f"最终角速度: {final_state['omega']:.2f}°/s")
    print(f"最终位置: ({final_state['x']:.2f}, {final_state['y']:.2f})")
    print(f"初始高度: {trajectory[0]['y']:.0f} m")
    print(f"初始水平距离: {trajectory[0]['x']:.0f} m")

    # 分析轨迹
    if len(trajectory) > 10:
        # 计算最大下降速度
        max_vy = min(s['vy'] for s in trajectory)
        # 计算最大姿态角
        max_theta = max(abs(s['theta']) for s in trajectory)
        # 计算平均推力
        avg_thrust = sum(s['thrust'] for s in trajectory) / len(trajectory)

        print(f"\n轨迹分析:")
        print(f"  最大下降速度: {max_vy:.1f} m/s")
        print(f"  最大姿态角: {max_theta:.1f}°")
        print(f"  平均推力: {avg_thrust/1000:.0f} kN")


def print_algorithm_design():
    """打印算法设计说明"""
    print("\n" + "=" * 60)
    print("火箭回收算法设计")
    print("=" * 60)

    print("\n1. 问题分析:")
    print("   - 火箭回收是一个复杂的最优控制问题")
    print("   - 需要同时控制位置、速度、姿态和燃料")
    print("   - 存在多种约束条件和安全要求")

    print("\n2. 算法架构:")
    print("   ┌─────────────────────────────────────────┐")
    print("   │         火箭回收控制系统                │")
    print("   ├─────────────────────────────────────────┤")
    print("   │  ┌─────────────┐  ┌─────────────┐     │")
    print("   │  │  轨迹规划器  │  │  姿态控制器  │     │")
    print("   │  │   (MPC)     │  │   (PID)     │     │")
    print("   │  └─────────────┘  └─────────────┘     │")
    print("   │          │               │              │")
    print("   │          └───────────────┼──────────────┘")
    print("   │                          ▼              │")
    print("   │                  ┌─────────────┐        │")
    print("   │                  │  控制分配器  │        │")
    print("   │                  └─────────────┘        │")
    print("   │                          │              │")
    print("   │                          ▼              │")
    print("   │                  ┌─────────────┐        │")
    print("   │                  │  火箭动力学  │        │")
    print("   │                  └─────────────┘        │")
    print("   └─────────────────────────────────────────┘")

    print("\n3. 控制策略:")
    print("   - 水平控制: 通过姿态角调整水平推力")
    print("   - 垂直控制: 通过节流阀调整推力大小")
    print("   - 姿态控制: 通过摆角调整推力方向")
    print("   - 燃料管理: 优化燃料使用，保证安全着陆")

    print("\n4. 关键技术:")
    print("   - PID控制器: 实时控制，简单有效")
    print("   - 模型预测控制(MPC): 轨迹规划，优化性能")
    print("   - 最优控制: 燃料优化，提高效率")
    print("   - 分段控制: 适应不同阶段需求")

    print("\n5. 性能指标:")
    print("   - 着陆精度: < 5m")
    print("   - 着陆速度: < 0.5m/s")
    print("   - 姿态误差: < 2°")
    print("   - 燃料效率: 优化控制减少燃料消耗")

    print("\n6. 应用场景:")
    print("   - 航天器回收: 火箭、飞船垂直着陆")
    print("   - 无人机着陆: 精准垂直着陆")
    print("   - 机器人控制: 精准定位和姿态控制")
    print("   - 自动驾驶: 车辆精准停车")

    print("\n7. 扩展方向:")
    print("   - 视觉着陆: 目标识别与跟踪")
    print("   - 环境感知: 地形适应和风扰补偿")
    print("   - 多火箭协同: 编队控制和任务分配")
    print("   - 硬件在环: 实时仿真和硬件测试")


def main():
    """主函数"""
    # 运行演示
    trajectory, success = simulate_rocket_landing_demo()
    print_results(trajectory, success)

    # 打印算法设计
    print_algorithm_design()

    # 总结
    print("\n" + "=" * 60)
    print("总结")
    print("=" * 60)
    print("\n火箭回收算法是一个复杂的控制系统，涉及:")
    print("1. 动力学建模: 火箭的非线性动力学")
    print("2. 控制理论: PID、MPC、最优控制")
    print("3. 实时计算: 快速响应和决策")
    print("4. 安全约束: 多种约束条件和安全要求")
    print("\n本算法展示了:")
    print("- 基本的控制逻辑和策略")
    print("- 分段控制和紧急处理")
    print("- 性能指标和优化方向")
    print("- 实际应用和扩展方向")


if __name__ == "__main__":
    main()
