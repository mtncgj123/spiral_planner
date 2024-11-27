from math import *
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as mpathes


class Plot:
    def __init__(self):

        self.block_x = 12.5
        self.block_y = 3.45
        self.block_width = 2
        self.block_height = 1.1
        self.block_angle = 0

        self.block_min_x = self.block_x-self.block_width/2
        self.block_max_x = self.block_x+self.block_width/2
        self.block_min_y = self.block_y-self.block_height/2
        self.block_max_y = self.block_y+self.block_height/2

    def block_plot(self):
        fig, ax = plt.subplots()
        plt.axis("equal")
        left_bottom_x = self.block_x-self.block_width/2
        left_bottom_y = self.block_y-self.block_height/2
        xy2 = np.array([left_bottom_x, left_bottom_y])
        rect = mpathes.Rectangle(
            xy2, self.block_width, self.block_height, color='black', angle=self.block_angle)
        ax.add_patch(rect)
        plt.plot()

    def forklift_draw(self, x, y, yaw, draw_flag):
        fork_anglepoint = [[0.75, 0.39], [
            0.75, -2], [-0.75, -2], [-0.75, 0.39]]
        spin_x = []
        spin_y = []
        rotation_angle = yaw-pi/2
        for point in fork_anglepoint:
            tmp_x = point[0]*cos(rotation_angle)-point[1]*sin(rotation_angle)
            tmp_y = point[0]*sin(rotation_angle)+point[1]*cos(rotation_angle)
            spin_x.append(tmp_x+x)
            spin_y.append(tmp_y+y)

        if draw_flag == 1:
            for i in range(len(spin_x)):
                if (i == len(spin_x)-1):
                    plt.plot((spin_x[i], spin_x[0]),
                             (spin_y[i], spin_y[0]), 'r')
                    break
                plt.plot((spin_x[i+1], spin_x[i]),
                         (spin_y[i+1], spin_y[i]), 'r')

            plt.arrow(x, y, 0.5*cos(yaw),
                      0.5*sin(yaw), color='r', width=0.1)

        return spin_x, spin_y

    def margin_plot(self, line_point1, line_point2):

        dir = np.array(line_point2)-np.array(line_point1)
        path_x = [line_point1[0]+dir[0]*t for t in np.arange(0, 1+0.1, 0.1)]
        path_y = [line_point1[1]+dir[1]*t for t in np.arange(0, 1+0.1, 0.1)]

        plt.plot(path_x, path_y, "--", color="black")

    # False:point invalid True:point valid
    def collision_check_line(self, x, y, line_point1, line_point2, valid_point):
        line_point1x = line_point1[0]
        line_point1y = line_point1[1]
        line_point2x = line_point2[0]
        line_point2y = line_point2[1]

        line_A = line_point2y-line_point1y
        line_B = line_point1x-line_point2x
        line_C = line_point2x*line_point1y-line_point1x*line_point2y
        flag = line_A*valid_point[0]+line_B*valid_point[1]+line_C
        if (line_A*x+line_B*y+line_C)*flag >= 0:
            return True
        else:
            return False

    # check-true False:have point valid True:all points invalid
    # check-false False:have point invalid True:all points valid
    def collision_check_line_forklift(self, x, y, yaw, line_point1, line_point2, valid_point, check):

        spin_x, spin_y = self.forklift_draw(x, y, yaw, -1)
        if (True == check):
            for i in range(len(spin_x)):
                if True == self.collision_check_line(spin_x[i], spin_y[i], line_point1, line_point2, valid_point):
                    return False
            return True
        else:
            for i in range(len(spin_x)):
                if False == self.collision_check_line(spin_x[i], spin_y[i], line_point1, line_point2, valid_point):
                    return False
            return True

    # true: all points in rect
    def collision_check_in_rect(self, x, y, yaw, rect):
        point_0 = rect[0]
        point_1 = rect[1]
        point_2 = rect[2]
        point_3 = rect[3]
        inner_point = (point_0+point_2)/2

        check0 = self.collision_check_line(
            x, y, yaw, point_0, point_1, inner_point, False)
        check1 = self.collision_check_line(
            x, y, yaw, point_1, point_2, inner_point, False)
        check2 = self.collision_check_line(
            x, y, yaw, point_2, point_3, inner_point, False)
        check3 = self.collision_check_line(
            x, y, yaw, point_3, point_0, inner_point, False)

        if True == (check0 and check1 and check2 and check3):
            return True
        else:
            return False

    # true: all points outside rect
    def collision_check_outside_rect(self, x, y, yaw, rect):
        point_0 = rect[0]
        point_1 = rect[1]
        point_2 = rect[2]
        point_3 = rect[3]
        inner_point = (point_0+point_2)/2

        check0 = self.collision_check_line_forklift(
            x, y, yaw, point_0, point_1, inner_point, True)
        check1 = self.collision_check_line_forklift(
            x, y, yaw, point_1, point_2, inner_point, True)
        check2 = self.collision_check_line_forklift(
            x, y, yaw, point_2, point_3, inner_point, True)
        check3 = self.collision_check_line_forklift(
            x, y, yaw, point_3, point_0, inner_point, True)

        if True == (check0 or check1 or check2 or check3):
            return True
        else:
            return False

    # 判断点是否在矩形内部（包括边界）。
    # :param point: 点的坐标 (x, y)
    # :param rect: 矩形的四点坐标 [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
    # :return: 布尔值，True 表示点在矩形内
    def is_point_in_rectangle(self, point, rect):

        x, y = point[0], point[1]
        x1, y1 = rect[0][0], rect[0][1]
        x2, y2 = rect[1][0], rect[1][1]
        x3, y3 = rect[2][0], rect[2][1]
        x4, y4 = rect[3][0], rect[3][1]

        # 利用向量叉积判断点是否在矩形内
        def cross_product(x1, y1, x2, y2, x3, y3):
            return (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)

        # 计算点是否在矩形内部
        d1 = cross_product(x1, y1, x2, y2, x, y)
        d2 = cross_product(x2, y2, x3, y3, x, y)
        d3 = cross_product(x3, y3, x4, y4, x, y)
        d4 = cross_product(x4, y4, x1, y1, x, y)

        return (d1 >= 0 and d2 >= 0 and d3 >= 0 and d4 >= 0) or \
            (d1 <= 0 and d2 <= 0 and d3 <= 0 and d4 <= 0)

    def do_lines_intersect(self, p1, p2, q1, q2):
        """
        判断两条线段是否相交。
        :param p1: 线段1的起点 (x1, y1)
        :param p2: 线段1的终点 (x2, y2)
        :param q1: 线段2的起点 (x3, y3)
        :param q2: 线段2的终点 (x4, y4)
        :return: 布尔值，True 表示相交
        """
        def orientation(p, q, r):
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if val == 0:
                return 0  # 共线
            return 1 if val > 0 else 2  # 顺时针 or 逆时针

        def on_segment(p, q, r):
            if min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[1] <= max(p[1], r[1]):
                return True
            return False

        o1 = orientation(p1, p2, q1)
        o2 = orientation(p1, p2, q2)
        o3 = orientation(q1, q2, p1)
        o4 = orientation(q1, q2, p2)

        # 检查一般相交情况
        if o1 != o2 and o3 != o4:
            return True

        # 检查特殊情况：共线但重叠
        if o1 == 0 and on_segment(p1, q1, p2):
            return True
        if o2 == 0 and on_segment(p1, q2, p2):
            return True
        if o3 == 0 and on_segment(q1, p1, q2):
            return True
        if o4 == 0 and on_segment(q1, p2, q2):
            return True

        return False

    def rectangles_intersect(self, rect1, rect2):
        """
        判断两个矩形是否相交。
        :param rect1: 矩形1的四点坐标 [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        :param rect2: 矩形2的四点坐标 [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        :return: 布尔值，True 表示两个矩形相交
        """
        # 检查是否有一条边相交
        for i in range(4):
            for j in range(4):
                if self.do_lines_intersect(rect1[i], rect1[(i + 1) % 4], rect2[j], rect2[(j + 1) % 4]):
                    return True

        # 检查一个矩形是否包含另一个矩形
        for point in rect1:
            if self.is_point_in_rectangle(point, rect2):
                return True
        for point in rect2:
            if self.is_point_in_rectangle(point, rect1):
                return True

        return False

    def collision_check_rect_forklift(self, forklift_x, forklift_y, forklift_yaw, rect):
        spin_x, spin_y = self.forklift_draw(
            forklift_x, forklift_y, forklift_yaw, -1)
        return self.rectangles_intersect(np.array([[spin_x[0], spin_y[0]], [spin_x[1], spin_y[1]], [spin_x[2], spin_y[2]], [spin_x[3], spin_y[3]]]), rect)

    def traj_collision_check(self, traj_x, traj_y, traj_yaw):
        for i in range(len(traj_x)):
            if self.collision_check_rect_forklift(traj_x[i], traj_y[i], traj_yaw[i], np.array([
                    [self.block_min_x, self.block_min_y], [self.block_min_x, self.block_max_y], [self.block_max_x, self.block_max_y], [self.block_max_x, self.block_min_y]])) == True:
                return True

        return False

    def closePlots(self):
        plt.clf()
        plt.cla()
        plt.close("all")


# if __name__ == "__main__":

#     pt = Plot()
#     pt.block_plot()

#     x_init = 13.0
#     y_init = 4.95
#     yaw_init = -170*pi/180
#     pt.forklift_draw(x_init, y_init, yaw_init, 1)

#     pt.margin_plot([7.59, 2.9], [19.07, 2.9])
#     pt.margin_plot([7.59, 2.9], [7.59, 8.5])
#     pt.margin_plot([7.59, 8.5], [19.07, 8.5])
#     pt.margin_plot([19.07, 8.5], [19.07, 2.9])

#     res = pt.collision_check_rect_forklift(x_init, y_init, yaw_init, np.array([
#         [pt.block_min_x, pt.block_min_y], [pt.block_min_x, pt.block_max_y], [pt.block_max_x, pt.block_max_y], [pt.block_max_x, pt.block_min_y]]))
#     print(res)

#     plt.show()
