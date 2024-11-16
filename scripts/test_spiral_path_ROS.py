from geometry_msgs.msg import Point, PoseArray
import rospy
import matplotlib.pyplot as plt
from math import *
from ROSSrv import *
from _spiral import spiral
from _circle import circle
from _line import line


if __name__ == "__main__":

    min_r = 2.5
    max_k = 1/min_r
    step_s = 0.01

    rospy.init_node("test_ros")
    client_spi = rospy.ServiceProxy("spiral_plan_request", spiral)
    client_spi.wait_for_service()

    client_circle = rospy.ServiceProxy("circle_plan_request", circle)
    client_circle.wait_for_service()

    client_line = rospy.ServiceProxy("line_plan_request", line)
    client_line.wait_for_service()

    x_init, y_init, yaw_init = 22.997613, 2.005575, 170*pi/180
    x_goal, y_goal, yaw_goal = 24.000000, 3.000000, 60*pi/180

    start_position = [x_init, y_init, yaw_init]
    goal_position = [x_goal, y_goal, yaw_goal]

    _start_position = Point()
    _goal_position = Point()
    traj = PoseArray()
    _start_position.x = start_position[0]
    _start_position.y = start_position[1]
    _start_position.z = start_position[2]
    _goal_position.x = goal_position[0]
    _goal_position.y = goal_position[1]
    _goal_position.z = goal_position[2]

    response = client_spi.call(_start_position, _goal_position)
    traj = response.traj
    polypath_x = []
    polypath_y = []
    polypath_yaw = []
    polypath_curv = []
    for i in range(len(traj.poses)):
        polypath_x.append(traj.poses[i].position.x)
        polypath_y.append(traj.poses[i].position.y)
        polypath_yaw.append(traj.poses[i].orientation.z)
        polypath_curv.append(traj.poses[i].orientation.w)
    polypath_s = [step_s*i for i in range(0, len(polypath_x))]
    min_curv_r = response.min_curvr
    s_flag = response.s_flag
    print("min curvr", min_curv_r)
    print("s_flag", s_flag)

    subplot = True

    if (subplot):
        # 创建一个图形和横向排列的子图
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

        # 绘制第一个图
        ax1.plot(polypath_x, polypath_y, 'g--')
        ax1.set_title('Spiral Path')
        ax1.set_xlabel('x')
        ax1.set_ylabel('y')
        ax1.set_aspect('equal', adjustable='box')  # 设置坐标轴比例相等
        ax1.legend(['Spiral Path'])

        # 绘制第二个图
        ax2.plot(polypath_s, polypath_curv, 'g--')
        ax2.axhline(y=max_k, color='red', linestyle='--', linewidth=2)  # 绘制虚线
        ax2.axhline(y=-max_k, color='red', linestyle='--', linewidth=2)  # 绘制虚线
        ax2.set_title('Curv')
        ax2.set_xlabel('s')
        ax2.set_ylabel('curv')
        ax2.legend(['Curvature'])
        ax2.set_ylim(-1.5, 1.5)

        # 调整布局
        plt.tight_layout()  # rect用于调整标题的位置

        # 显示图形
        plt.show()
    else:
        # 绘制路径
        plt.plot(polypath_x, polypath_y, 'g--')
        plt.axhline(y=max_k, color='red', linestyle='--', linewidth=2)  # 绘制虚线
        plt.axhline(y=-max_k, color='red', linestyle='--', linewidth=2)  # 绘制虚线
        plt.title('Spiral Path')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis('equal')
        plt.legend(['Spiral Path'])
        plt.show()
