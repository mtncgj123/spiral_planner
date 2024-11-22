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

    def collision_check_line(self, x, y, yaw, line_point1, line_point2, valid_point):
        line_point1x = line_point1[0]
        line_point1y = line_point1[1]
        line_point2x = line_point2[0]
        line_point2y = line_point2[1]

        line_A = line_point2y-line_point1y
        line_B = line_point1x-line_point2x
        line_C = line_point2x*line_point1y-line_point1x*line_point2y
        flag = line_A*valid_point[0]+line_B*valid_point[1]+line_C
        spin_x, spin_y = self.forklift_draw(x, y, yaw, -1)
        for i in range(len(spin_x)):
            if (line_A*spin_x[i]+line_B*spin_y[i]+line_C)*flag < 0:
                return False
        return True

    def collision_check_rect(self, x, y, yaw, rect):
        point_0 = rect[0]
        point_1 = rect[1]
        point_2 = rect[2]
        point_3 = rect[3]
        valid_point = (point_0+point_2)/2

        if False == self.collision_check_line(x, y, yaw, point_0, point_1, valid_point):
            return False
        if False == self.collision_check_line(x, y, yaw, point_1, point_2, valid_point):
            return False
        if False == self.collision_check_line(x, y, yaw, point_2, point_3, valid_point):
            return False
        if False == self.collision_check_line(x, y, yaw, point_3, point_0, valid_point):
            return False
        return True

    def traj_collision_check(self, traj_x, traj_y, traj_yaw, line_point1, line_point2, valid_point):
        for i in range(len(traj_x)):
            if self.collision_check(traj_x[i], traj_y[i], traj_yaw[i], line_point1, line_point2, valid_point) == False:
                return False

        return True

    def closePlots(self):
        plt.clf()
        plt.cla()
        plt.close("all")


if __name__ == "__main__":

    pt = Plot()
    pt.block_plot()

    x_init = 12.5
    y_init = 4.65
    yaw_init = 180*pi/180
    pt.forklift_draw(x_init, y_init, yaw_init, 1)

    pt.margin_plot([7.59, 2.9], [19.07, 2.9])
    pt.margin_plot([7.59, 2.9], [7.59, 8.5])
    pt.margin_plot([7.59, 8.5], [19.07, 8.5])
    pt.margin_plot([19.07, 8.5], [19.07, 2.9])

    res = pt.collision_check_rect(x_init, y_init, yaw_init, np.array([
                                  [11.5, 2.9], [13.5, 2.9], [13.5, 4], [11.5, 4]]))
    print(res)

    plt.show()
