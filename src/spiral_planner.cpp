#include "Spiral_Planner.h"

#include <math.h>
#include <ros/ros.h>

#include <algorithm>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <typeinfo>
#include <vector>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "spiral_planner/circle.h"
#include "spiral_planner/line.h"
#include "spiral_planner/spiral.h"

using namespace std;
using CppAD::AD;

Short_Distance_Planner sdp;

// FG_eval function definition

FG_eval::FG_eval(Position _start_position, Position _goal_position)
{
    start_position = _start_position;
    goal_position = _goal_position;
}

AD<double> FG_eval::curv(const AD<double>& x0, const AD<double>& x1, const AD<double>& x2, const AD<double>& x3,
                         const AD<double>& s)
{
    return x3 * pow(s, 3) + x2 * pow(s, 2) + x1 * s + x0;
}

AD<double> FG_eval::theta(const AD<double> theta_0, const AD<double>& x0, const AD<double>& x1, const AD<double>& x2,
                          const AD<double>& x3, const AD<double>& s)
{
    return theta_0 + x3 * pow(s, 4) / 4 + x2 * pow(s, 3) / 3 + x1 * pow(s, 2) / 2 + x0 * s;
}

AD<double> FG_eval::position_x(const AD<double> position_x0, const AD<double> theta_0, const AD<double>& x0,
                               const AD<double>& x1, const AD<double>& x2, const AD<double>& x3, const AD<double>& s)
{
    return position_x0 +
           s / 24 *
               (cos(theta(theta_0, x0, x1, x2, x3, 0)) + 4 * cos(theta(theta_0, x0, x1, x2, x3, s / 8)) +
                2 * cos(theta(theta_0, x0, x1, x2, x3, s * 2 / 8)) +
                4 * cos(theta(theta_0, x0, x1, x2, x3, s * 3 / 8)) +
                2 * cos(theta(theta_0, x0, x1, x2, x3, s * 4 / 8)) +
                4 * cos(theta(theta_0, x0, x1, x2, x3, s * 5 / 8)) +
                2 * cos(theta(theta_0, x0, x1, x2, x3, s * 6 / 8)) +
                4 * cos(theta(theta_0, x0, x1, x2, x3, s * 7 / 8)) + cos(theta(theta_0, x0, x1, x2, x3, s * 8 / 8)));
}

AD<double> FG_eval::position_y(const AD<double> position_y0, const AD<double> theta_0, const AD<double>& x0,
                               const AD<double>& x1, const AD<double>& x2, const AD<double>& x3, const AD<double>& s)
{
    return position_y0 +
           s / 24 *
               (sin(theta(theta_0, x0, x1, x2, x3, 0)) + 4 * sin(theta(theta_0, x0, x1, x2, x3, s / 8)) +
                2 * sin(theta(theta_0, x0, x1, x2, x3, s * 2 / 8)) +
                4 * sin(theta(theta_0, x0, x1, x2, x3, s * 3 / 8)) +
                2 * sin(theta(theta_0, x0, x1, x2, x3, s * 4 / 8)) +
                4 * sin(theta(theta_0, x0, x1, x2, x3, s * 5 / 8)) +
                2 * sin(theta(theta_0, x0, x1, x2, x3, s * 6 / 8)) +
                4 * sin(theta(theta_0, x0, x1, x2, x3, s * 7 / 8)) + sin(theta(theta_0, x0, x1, x2, x3, s * 8 / 8)));
}

AD<double> FG_eval::cost(const AD<double>& x10, const AD<double>& x11, const AD<double>& x12, const AD<double>& x13,
                         const AD<double>& x14, const AD<double>& x20, const AD<double>& x21, const AD<double>& x22,
                         const AD<double>& x23, const AD<double>& x24)
{
    // double x_init = start_position.x;
    // double y_init = start_position.y;
    // double yaw_init = start_position.yaw;

    // double x_goal = goal_position.x;
    // double y_goal = goal_position.y;
    // double yaw_goal = goal_position.yaw;

    return (pow(x23, 2) / 7 * pow(x24, 7) + 2 / 6 * x23 * x22 * pow(x24, 6) +
            (pow(x22, 2) + 2 * x23 * x21 + 9 * pow(x23, 2)) / 5 * pow(x24, 5) +
            (2 * x23 * x20 + 2 * x22 * x21 + 12 * x22 * x23) / 4 * pow(x24, 4) +
            (pow(x21, 2) + 2 * x22 * x20 + 4 * pow(x22, 2) + 6 * x21 * x23) / 3 * pow(x24, 3) +
            (2 * x21 * x20 + 4 * x21 * x22) / 2 * pow(x24, 2) + (pow(x20, 2) + pow(x21, 2)) * x24) +
           (pow(x13, 2) / 7 * pow(x14, 7) + 2 / 6 * x13 * x12 * pow(x14, 6) +
            (pow(x12, 2) + 2 * x13 * x11 + 9 * pow(x13, 2)) / 5 * pow(x14, 5) +
            (2 * x13 * x10 + 2 * x12 * x11 + 12 * x12 * x13) / 4 * pow(x14, 4) +
            (pow(x11, 2) + 2 * x12 * x10 + 4 * pow(x12, 2) + 6 * x11 * x13) / 3 * pow(x14, 3) +
            (2 * x11 * x10 + 4 * x11 * x12) / 2 * pow(x14, 2) + (pow(x10, 2) + pow(x11, 2)) * x14) +
           0.5 * pow(x14, 2) + 0.5 * pow(x24, 2);

    // return (pow(x23, 2) / 7 * pow(x24, 7) + pow(x22, 2) / 5 * pow(x24, 5) + pow(x21, 2) / 3 * pow(x24, 3) + pow(x20,
    // 2) * x24 +
    //         2 / 6 * x22 * x23 * pow(x24, 6) + 2 / 4 * x21 * x22 * pow(x24, 4) + 2 / 3 * x20 * x22 * pow(x24, 3) +
    //         2 / 5 * x23 * x21 * pow(x24, 5) + 2 / 4 * x23 * x20 * pow(x24, 4) + x21 * x20 * pow(x24, 2) +
    //         pow(x13, 2) / 7 * pow(x14, 7) + pow(x12, 2) / 5 * pow(x14, 5) + pow(x11, 2) / 3 * pow(x14, 3) + pow(x10,
    //         2) * x14 + 2 / 6 * x12 * x13 * pow(x14, 6) + 2 / 4 * x11 * x12 * pow(x14, 4) + 2 / 3 * x10 * x12 *
    //         pow(x14, 3) + 2 / 5 * x13 * x11 * pow(x14, 5) + 2 / 4 * x13 * x10 * pow(x14, 4) + x11 * x10 * pow(x14,
    //         2)) +
    //        1 * pow(x14, 2) + 1 * pow(x24, 2);

    // 100 * pow(position_x(x_init, yaw_init, x0, x1, x2, x3, x4) - x_goal, 2) +
    // 100 * pow(position_y(y_init, yaw_init, x0, x1, x2, x3, x4) - y_goal, 2) +
    // 100 * pow(theta(yaw_init, x0, x1, x2, x3, x4) - yaw_goal, 2));
}

vector<AD<double>> FG_eval::mapping_p2x(const AD<double>& p0, const AD<double>& p1, const AD<double>& p2,
                                        const AD<double>& p3, const AD<double>& p4)
{
    vector<AD<double>> res;
    AD<double> x1 = -(11 / 2 * p0 - 9 * p1 + 9 / 2 * p2 - p3) / p4;
    AD<double> x2 = (9 * p0 - 45 / 2 * p1 + 18 * p2 - 9 / 2 * p3) / pow(p4, 2);
    AD<double> x3 = -(9 / 2 * p0 - 27 / 2 * p1 + 27 / 2 * p2 - 9 / 2 * p3) / pow(p4, 3);
    res.emplace_back(p0);
    res.emplace_back(x1);
    res.emplace_back(x2);
    res.emplace_back(x3);
    res.emplace_back(p4);
    return res;
}

void FG_eval::operator()(ADvector& fg, const ADvector& p)
{
    assert(fg.size() == 15);
    assert(p.size() == 13);
    // variables
    AD<double> p0 = p[0];
    AD<double> p1 = p[1];
    AD<double> p2 = p[2];
    AD<double> p3 = p[3];
    AD<double> p4 = p[4];

    AD<double> p5 = p[5];
    AD<double> p6 = p[6];
    AD<double> p7 = p[7];
    AD<double> p8 = p[8];
    AD<double> p9 = p[9];

    AD<double> p10 = p[10];
    AD<double> p11 = p[11];
    AD<double> p12 = p[12];
    // f(x) objective function

    vector<AD<double>> x_set1 = mapping_p2x(p0, p1, p2, p3, p4);
    vector<AD<double>> x_set2 = mapping_p2x(p5, p6, p7, p8, p9);

    fg[0] = cost(x_set1[0], x_set1[1], x_set1[2], x_set1[3], x_set1[4], x_set2[0], x_set2[1], x_set2[2], x_set2[3],
                 x_set2[4]);
    // constraints
    fg[1] = p0;
    fg[2] = pow(max_curv, 2) - pow(p1, 2);
    fg[3] = pow(max_curv, 2) - pow(p2, 2);
    fg[4] = pow(max_curv, 2) - pow(p3, 2);

    fg[5] = p5;
    fg[6] = pow(max_curv, 2) - pow(p6, 2);
    fg[7] = pow(max_curv, 2) - pow(p7, 2);
    fg[8] = pow(max_curv, 2) - pow(p8, 2);

    fg[9] =
        position_x(start_position.x, start_position.yaw, x_set1[0], x_set1[1], x_set1[2], x_set1[3], x_set1[4]) - p10;
    fg[10] =
        position_y(start_position.y, start_position.yaw, x_set1[0], x_set1[1], x_set1[2], x_set1[3], x_set1[4]) - p11;
    fg[11] = theta(start_position.yaw, x_set1[0], x_set1[1], x_set1[2], x_set1[3], x_set1[4]) - p12;

    fg[12] = position_x(p10, p12, x_set2[0], x_set2[1], x_set2[2], x_set2[3], x_set2[4]) - goal_position.x;
    fg[13] = position_y(p11, p12, x_set2[0], x_set2[1], x_set2[2], x_set2[3], x_set2[4]) - goal_position.y;
    fg[14] = theta(p12, x_set2[0], x_set2[1], x_set2[2], x_set2[3], x_set2[4]) - goal_position.yaw;
}

// Short_Distance_Planner function definition

// circle path plan
int Short_Distance_Planner::circle_origin_flag(Position start_position, Position goal_position)
{
    vector<double> start_dir = {cos(start_position.yaw), sin(start_position.yaw)};
    vector<double> goal_dir = {cos(goal_position.yaw), sin(goal_position.yaw)};

    if (start_dir[0] * goal_dir[1] - start_dir[1] * goal_dir[0] >= 0)
    {
        return -1;
    }
    return 1;
}

Position Short_Distance_Planner::circle_origin_finder(Position start_position, double r, int flag)
{
    double start_position_x = start_position.x;
    double start_position_y = start_position.y;
    double start_position_yaw = start_position.yaw;

    double theta_origin = start_position_yaw + M_PI / 2 * flag;
    double origin_x = start_position_x + r * cos(theta_origin);
    double origin_y = start_position_y + r * sin(theta_origin);
    Position origin = {origin_x, origin_y, 0};

    return origin;
}

vector<Position> Short_Distance_Planner::circle_path_finder(Position start_position, Position origin_position,
                                                            double yaw_inter, int flag)
{
    double theta_ori2goal = yaw_inter - M_PI / 2 * flag;
    double rotation_direction;
    vector<Position> circle_path;

    double start_position_x = start_position.x;
    double start_position_y = start_position.y;
    double start_position_yaw = start_position.yaw;

    double origin_x = origin_position.x;
    double origin_y = origin_position.y;

    vector<double> dir_ori2start = {start_position_x - origin_x, start_position_y - origin_y};
    vector<double> dir_ori2goal = {cos(theta_ori2goal), sin(theta_ori2goal)};

    double theta_rotation = acos((dir_ori2start[0] * dir_ori2goal[0] + dir_ori2start[1] * dir_ori2goal[1]) /
                                 (sqrt(pow(dir_ori2start[0], 2) + pow(dir_ori2start[1], 2)) *
                                  sqrt(pow(dir_ori2goal[0], 2) + pow(dir_ori2goal[1], 2))));
    if (dir_ori2start[0] * dir_ori2goal[1] - dir_ori2start[1] * dir_ori2goal[0] > 0)
    {
        rotation_direction = 1;
    }
    else
    {
        rotation_direction = -1;
    }

    double n_theta_step = 100.0;
    Position p;
    for (int i = 0; i < n_theta_step + 1; i++)
    {
        p.x = origin_x + cos(i / n_theta_step * theta_rotation * rotation_direction) * (start_position_x - origin_x) -
              sin(i / n_theta_step * theta_rotation * rotation_direction) * (start_position_y - origin_y);

        p.y = origin_y + sin(i / n_theta_step * theta_rotation * rotation_direction) * (start_position_x - origin_x) +
              cos(i / n_theta_step * theta_rotation * rotation_direction) * (start_position_y - origin_y);

        p.yaw = start_position_yaw + i / n_theta_step * theta_rotation * rotation_direction;

        circle_path.push_back(p);
    }
    return circle_path;
}

// line path plan

vector<Position> Short_Distance_Planner::line_path_finder(Position start_position, int direction, double distance)
{
    double start_position_x = start_position.x;
    double start_position_y = start_position.y;
    double start_position_yaw = start_position.yaw;
    vector<Position> line_path;
    double moving_yaw;

    if (direction == -1)
    {
        moving_yaw = start_position_yaw + M_PI;
    }
    else
    {
        moving_yaw = start_position_yaw;
    }
    double path_step = 0.3;
    int n_path_step = (int)distance / path_step;
    for (int i = 0; i < n_path_step + 1; i++)
    {
        Position p;
        p.x = start_position_x + i * path_step * cos(moving_yaw);
        p.y = start_position_y + i * path_step * sin(moving_yaw);
        p.yaw = start_position_yaw;
        line_path.push_back(p);
    }
    return line_path;
}

// spiral path plan

vector<double> Short_Distance_Planner::cumulative_trapezoid(vector<double> x_discrete, vector<double> s_set,
                                                            double initial)
{
    vector<double> cumulative_res;
    double sum = initial;
    cumulative_res.push_back(sum);
    for (int i = 0; i < s_set.size() - 1; i++)
    {
        sum += (x_discrete[i] + x_discrete[i + 1]) / 2 * (s_set[i + 1] - s_set[i]);
        cumulative_res.push_back(sum);
    }
    return cumulative_res;
}

tuple<vector<double>, vector<double>, vector<double>, double, bool>
    Short_Distance_Planner::spiral_path_finder(Position start_position, Position goal_position)
{
    // bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    size_t nx = 13;  // number of varibles
    size_t ng = 14;  // number of constraints
    Dvector p0(nx);  // initial condition of varibles
    p0[0] = 0.0;
    p0[1] = 0.0;
    p0[2] = 0.0;
    p0[3] = 0.0;
    p0[4] = 10.0;

    p0[5] = 0.0;
    p0[6] = 0.0;
    p0[7] = 0.0;
    p0[8] = 0.0;
    p0[9] = 10.0;

    p0[10] = 0.0;
    p0[11] = 0.0;
    p0[12] = 0.0;

    // lower and upper bounds for varibles
    Dvector xl(nx), xu(nx);
    for (i = 0; i < nx; i++)
    {
        xl[i] = -100;
        xu[i] = +100;
    }
    xl[12] = -M_PI;
    xu[12] = +M_PI;

    Dvector gl(ng), gu(ng);
    gl[0] = 0;
    gu[0] = 0;
    gl[1] = 0;
    gu[1] = 1.0e19;
    gl[2] = 0;
    gu[2] = 1.0e19;
    gl[3] = 0;
    gu[3] = 1.0e19;

    gl[4] = 0;
    gu[4] = 0;
    gl[5] = 0;
    gu[5] = 1.0e19;
    gl[6] = 0;
    gu[6] = 1.0e19;
    gl[7] = 0;
    gu[7] = 1.0e19;

    gl[8] = 0;
    gu[8] = 0;
    gl[9] = 0;
    gu[9] = 0;
    gl[10] = 0;
    gu[10] = 0;

    gl[11] = 0;
    gu[11] = 0;
    gl[12] = 0;
    gu[12] = 0;
    gl[13] = 0;
    gu[13] = 0;

    // object that computes objective and constraints
    // ptr_fg_eval= new FG_eval(start_position, goal_position);

    // options
    string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String sb            yes\n";
    // maximum iterations
    options += "Integer max_iter     100\n";
    // approximate accuracy in first order necessary conditions;
    //  see Mathematical Programming, Volume 106, Number 1,
    //  Pages 25-57, Equation (6)
    options += "Numeric tol          1e-6\n";
    // derivative tesing
    options += "String derivative_test   second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius   0.\n";

    CppAD::ipopt::solve_result<Dvector> solution;  // solution
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, p0, xl, xu, gl, gu, *ptr_fg_eval, solution);

    // cout << "solution: " << solution.x << endl;
    vector<AD<double>> res_x1 =
        ptr_fg_eval->mapping_p2x((solution.x)[0], (solution.x)[1], (solution.x)[2], (solution.x)[3], (solution.x)[4]);
    vector<AD<double>> res_x2 =
        ptr_fg_eval->mapping_p2x((solution.x)[5], (solution.x)[6], (solution.x)[7], (solution.x)[8], (solution.x)[9]);

    double s = CppAD::Value(res_x1[4]);
    double step = 0.01;
    vector<double> x_discrete1, y_discrete1, spiral_path_x1, spiral_path_y1, spiral_path_yaw1, spiral_path_curvr1,
        s_set1;
    if (s > 0)
    {
        for (double i = 0; i < s + step; i += step)
        {
            double theta_yaw =
                CppAD::Value(ptr_fg_eval->theta(start_position.yaw, res_x1[0], res_x1[1], res_x1[2], res_x1[3], i));
            x_discrete1.push_back(cos(theta_yaw));
            y_discrete1.push_back(sin(theta_yaw));
            spiral_path_yaw1.push_back(theta_yaw);
            s_set1.push_back(i);
            spiral_path_curvr1.push_back(
                1.0 / max(fabs(CppAD::Value(ptr_fg_eval->curv(res_x1[0], res_x1[1], res_x1[2], res_x1[3], i))), 0.01));
        }
    }
    else
    {
        for (double i = 0; i > s - step; i -= step)
        {
            double theta_yaw =
                CppAD::Value(ptr_fg_eval->theta(start_position.yaw, res_x1[0], res_x1[1], res_x1[2], res_x1[3], i));
            x_discrete1.push_back(cos(theta_yaw));
            y_discrete1.push_back(sin(theta_yaw));
            spiral_path_yaw1.push_back(theta_yaw);
            s_set1.push_back(i);
            spiral_path_curvr1.push_back(
                1.0 / max(fabs(CppAD::Value(ptr_fg_eval->curv(res_x1[0], res_x1[1], res_x1[2], res_x1[3], i))), 0.01));
        }
    }
    spiral_path_x1 = cumulative_trapezoid(x_discrete1, s_set1, start_position.x);
    spiral_path_y1 = cumulative_trapezoid(y_discrete1, s_set1, start_position.y);
    double min_r1 = *min_element(spiral_path_curvr1.begin(), spiral_path_curvr1.end());

    s = CppAD::Value(res_x2[4]);
    step = 0.01;
    vector<double> x_discrete2, y_discrete2, spiral_path_x2, spiral_path_y2, spiral_path_yaw2, spiral_path_curvr2,
        s_set2;
    if (s > 0)
    {
        for (double i = 0; i < s + step; i += step)
        {
            double theta_yaw =
                CppAD::Value(ptr_fg_eval->theta(solution.x[12], res_x2[0], res_x2[1], res_x2[2], res_x2[3], i));
            x_discrete2.push_back(cos(theta_yaw));
            y_discrete2.push_back(sin(theta_yaw));
            spiral_path_yaw2.push_back(theta_yaw);
            s_set2.push_back(i);
            spiral_path_curvr2.push_back(
                1.0 / max(fabs(CppAD::Value(ptr_fg_eval->curv(res_x2[0], res_x2[1], res_x2[2], res_x2[3], i))), 0.01));
        }
    }
    else
    {
        for (double i = 0; i > s - step; i -= step)
        {
            double theta_yaw =
                CppAD::Value(ptr_fg_eval->theta(solution.x[12], res_x2[0], res_x2[1], res_x2[2], res_x2[3], i));
            x_discrete2.push_back(cos(theta_yaw));
            y_discrete2.push_back(sin(theta_yaw));
            spiral_path_yaw2.push_back(theta_yaw);
            s_set2.push_back(i);
            spiral_path_curvr2.push_back(
                1.0 / max(fabs(CppAD::Value(ptr_fg_eval->curv(res_x2[0], res_x2[1], res_x2[2], res_x2[3], i))), 0.01));
        }
    }
    spiral_path_x2 = cumulative_trapezoid(x_discrete2, s_set2, solution.x[10]);
    spiral_path_y2 = cumulative_trapezoid(y_discrete2, s_set2, solution.x[11]);
    double min_r2 = *min_element(spiral_path_curvr2.begin(), spiral_path_curvr2.end());

    double min_r = min(min_r1, min_r2);

    vector<double> spiral_path_x, spiral_path_y, spiral_path_yaw;
    spiral_path_x.insert(spiral_path_x.end(), spiral_path_x1.begin(), spiral_path_x1.end());
    spiral_path_y.insert(spiral_path_y.end(), spiral_path_y1.begin(), spiral_path_y1.end());
    spiral_path_yaw.insert(spiral_path_yaw.end(), spiral_path_yaw1.begin(), spiral_path_yaw1.end());
    spiral_path_x.insert(spiral_path_x.end(), spiral_path_x2.begin(), spiral_path_x2.end());
    spiral_path_y.insert(spiral_path_y.end(), spiral_path_y2.begin(), spiral_path_y2.end());
    spiral_path_yaw.insert(spiral_path_yaw.end(), spiral_path_yaw2.begin(), spiral_path_yaw2.end());

    // bool s_flag;
    // bool s_cost = s < s_cost_coef * sqrt(pow(start_position.x - goal_position.x, 2) + pow(start_position.y -
    // goal_position.y, 2)); bool goal_cost = sqrt(pow(spiral_path_x.back() - goal_position.x, 2) +
    // pow(spiral_path_y.back() - goal_position.y, 2) + pow(spiral_path_yaw.back() - goal_position.yaw, 2)) <
    // goal_cost_tolerence;

    // if (s_cost==true && goal_cost==true)
    // {
    //     s_flag = true;
    // }
    // else
    // {
    //     s_flag = false;
    // }

    return make_tuple(spiral_path_x, spiral_path_y, spiral_path_yaw, min_r, true);
}

// short path plan
// vector<double> Short_Distance_Planner::short_path_finder(Position start_position, Position goal_position)
// {
//     int origin_flag=circle_origin_flag(start_position,goal_position);
//     double circle_path_yawset_end=start_position.yaw-origin_flag*135/180*M_PI;
//     vector<Position>circle_path;
//     for(double r=circle_path_r_max;r>circle_path_r_min-0.5;r-=0.5)
//     {
//         for(double
//         circle_path_goal_yaw=start_position.yaw;circle_path_goal_yaw>circle_path_goal_yaw-origin_flag*10/180*M_PI;circle_path_goal_yaw-=-origin_flag*10/180*M_PI)
//         {
//             for(double distance=0;distance<3;distance+=0.5)
//             {
//                 Position origin_position=circle_origin_finder(start_position,r,origin_flag);
//                 circle_path=circle_path_finder(start_position,origin_position,circle_path_goal_yaw,origin_flag);

// }
// }
// }

// }

//********* callback function **************//

bool spiral_plan_srv_callback(spiral_planner::spiral::Request& request, spiral_planner::spiral::Response& response)
{
    geometry_msgs::Point p1 = request.start_position;
    geometry_msgs::Point p2 = request.goal_position;

    Position _start_position(p1.x, p1.y, p1.z);
    Position _goal_position(p2.x, p2.y, p2.z);
    sdp.ptr_fg_eval->start_position = _start_position;
    sdp.ptr_fg_eval->goal_position = _goal_position;
    tuple<vector<double>, vector<double>, vector<double>, double, bool> res =
        sdp.spiral_path_finder(_start_position, _goal_position);
    vector<double> spiral_path_x, spiral_path_y, spiral_path_yaw;
    double min_curvr;
    bool s_flag;
    tie(spiral_path_x, spiral_path_y, spiral_path_yaw, min_curvr, s_flag) = res;

    geometry_msgs::PoseArray traj;
    for (int i = 0; i < (int)spiral_path_x.size(); i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = spiral_path_x[i];
        pose.position.y = spiral_path_y[i];
        pose.position.z = spiral_path_yaw[i];
        traj.poses.push_back(pose);
    }
    response.traj = traj;

    response.min_curvr = min_curvr;
    response.s_flag = s_flag;

    return true;
}

bool circle_plan_srv_callback(spiral_planner::circle::Request& request, spiral_planner::circle::Response& response)
{
    geometry_msgs::Point p1 = request.start_position;
    geometry_msgs::Point p2 = request.goal_position;
    double yaw_inter = request.yaw_inter;
    double r = request.r;

    Position _start_position(p1.x, p1.y, p1.z);
    Position _goal_position(p2.x, p2.y, p2.z);
    int flag = sdp.circle_origin_flag(_start_position, _goal_position);
    Position _origin_position = sdp.circle_origin_finder(_start_position, r, flag);

    vector<Position> circle_path = sdp.circle_path_finder(_start_position, _origin_position, yaw_inter, flag);
    geometry_msgs::PoseArray traj;
    for (int i = 0; i < (int)circle_path.size(); i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = circle_path[i].x;
        pose.position.y = circle_path[i].y;
        pose.position.z = circle_path[i].yaw;
        // cout << "x= " << circle_path[i].x << " y=" << circle_path[i].y << " z= " << circle_path[i].yaw << endl;
        traj.poses.push_back(pose);
    }
    response.traj = traj;

    return true;
}

bool line_plan_srv_callback(spiral_planner::line::Request& request, spiral_planner::line::Response& response)
{
    geometry_msgs::Point p1 = request.start_position;
    int direction = request.direction;
    double distance = request.distance;

    Position _start_position(p1.x, p1.y, p1.z);

    vector<Position> line_path = sdp.line_path_finder(_start_position, direction, distance);
    geometry_msgs::PoseArray traj;
    for (int i = 0; i < (int)line_path.size(); i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = line_path[i].x;
        pose.position.y = line_path[i].y;
        pose.position.z = line_path[i].yaw;
        traj.poses.push_back(pose);
    }
    response.traj = traj;

    return true;
}

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "spiral_planner");
//     ros::NodeHandle nh;
//     sdp.ptr_fg_eval = new FG_eval();

// ros::ServiceServer server_spi_planner = nh.advertiseService("spiral_plan_request", spiral_plan_srv_callback);
// ros::ServiceServer server_circle_planner = nh.advertiseService("circle_plan_request", circle_plan_srv_callback);
// ros::ServiceServer server_line_planner = nh.advertiseService("line_plan_request", line_plan_srv_callback);
// ros::spin();

// return 0;
// }

int main()
{
    Position _start_position(30, 5, 90.0 / 180.0 * M_PI);
    Position _goal_position(28.8, 5, 117.0 / 180.0 * M_PI);

    tuple<vector<double>, vector<double>, vector<double>, double, bool> res =
        sdp.spiral_path_finder(_start_position, _goal_position);
    vector<double> spiral_path_x, spiral_path_y, spiral_path_yaw;
    double min_curvr;
    bool s_flag;
    tie(spiral_path_x, spiral_path_y, spiral_path_yaw, min_curvr, s_flag) = res;
    cout << min_curvr << endl;
    cout << s_flag << endl;

    return 0;
}