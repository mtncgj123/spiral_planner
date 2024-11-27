from geometry_msgs.msg import Point, PoseArray
import rospy
import matplotlib.pyplot as plt
from math import *
from ROSSrv import *
from plot import *
from _spiral import spiral


class Position:
    def __init__(self, position_x, position_y, position_yaw):
        self.x = position_x
        self.y = position_y
        self.yaw = position_yaw


if __name__ == "__main__":

    min_r = 2.5
    max_k = 1/min_r
    step_s = 0.01

    rospy.init_node("test_ros")
    client_spi = rospy.ServiceProxy("spiral_plan_request", spiral)
    client_spi.wait_for_service()

    start_position_list = []
    goal_position_list = []

    start_position_list.append(Position(13.20, 4.80, 175*pi/180))  # 0
    goal_position_list.append(Position(10, 4.85, 180*pi/180))

    test_index = 0
    start_position = [start_position_list[test_index].x,
                      start_position_list[test_index].y, start_position_list[test_index].yaw]
    goal_position = [goal_position_list[test_index].x,
                     goal_position_list[test_index].y, goal_position_list[test_index].yaw]

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

    pt = Plot()
    pt.block_plot()

    pt.forklift_draw(start_position_list[test_index].x,
                     start_position_list[test_index].y, start_position_list[test_index].yaw, 1)

    pt.margin_plot([7.59, 2.9], [19.07, 2.9])
    pt.margin_plot([7.59, 2.9], [7.59, 8.5])
    pt.margin_plot([7.59, 8.5], [19.07, 8.5])
    pt.margin_plot([19.07, 8.5], [19.07, 2.9])

    res = pt.traj_collision_check(polypath_x, polypath_y, polypath_yaw)
    if res:
        print("Collision detected!")
    else:
        print("No collision!")

    plt.plot(polypath_x, polypath_y, 'g--')

    plt.show()
