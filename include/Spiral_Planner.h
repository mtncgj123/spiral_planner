#ifndef _SPIRAL_PLANNER_H
#    define _SPIRAL_PALNNER_H

#    include <cppad/ipopt/solve.hpp>

using CppAD::AD;
using namespace std;

struct Position
{
    double x;
    double y;
    double yaw;

    Position() {}

    Position(double _x, double _y, double _yaw)
    {
        x = _x;
        y = _y;
        yaw = _yaw;
    }
};

class FG_eval
{
  public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    Position start_position;
    Position goal_position;

    double min_curv_r = 2.8;
    double max_curv = 1 / min_curv_r;

    FG_eval(){};
    FG_eval(Position _start_position, Position _goal_position);
    AD<double> curv(const AD<double>& x0, const AD<double>& x1, const AD<double>& x2, const AD<double>& x3,
                    const AD<double>& s);
    AD<double> theta(const AD<double> theta_0, const AD<double>& x0, const AD<double>& x1, const AD<double>& x2,
                     const AD<double>& x3, const AD<double>& s);
    AD<double> position_x(const AD<double> position_x0, const AD<double> theta_0, const AD<double>& x0,
                          const AD<double>& x1, const AD<double>& x2, const AD<double>& x3, const AD<double>& s);
    AD<double> position_y(const AD<double> position_y0, const AD<double> theta_0, const AD<double>& x0,
                          const AD<double>& x1, const AD<double>& x2, const AD<double>& x3, const AD<double>& s);
    AD<double> cost(const AD<double>& x10, const AD<double>& x11, const AD<double>& x12, const AD<double>& x13,
                    const AD<double>& x14, const AD<double>& x20, const AD<double>& x21, const AD<double>& x22,
                    const AD<double>& x23, const AD<double>& x24);
    vector<AD<double>> mapping_p2x(const AD<double>& p0, const AD<double>& p1, const AD<double>& p2,
                                   const AD<double>& p3, const AD<double>& p4);
    void operator()(ADvector& fg, const ADvector& p);
};

class Short_Distance_Planner
{
  public:
    double s_cost_coef = 2;
    double goal_cost_tolerence = 0.1;
    double circle_path_r_max = 5.0;
    double circle_path_r_min = 3.0;

    FG_eval* ptr_fg_eval;

    // circle path plan
    int circle_origin_flag(Position start_position, Position goal_position);
    Position circle_origin_finder(Position start_position, double r, int flag);
    vector<Position> circle_path_finder(Position start_position, Position origin_position, double yaw_inter, int flag);

    // line path plan
    vector<Position> line_path_finder(Position start_positio, int direction, double distance);

    // spiral path plan
    vector<double> cumulative_trapezoid(vector<double> x_discrete, vector<double> s_set, double initial);
    tuple<vector<double>, vector<double>, vector<double>, double, bool> spiral_path_finder(Position start_position,
                                                                                           Position goal_position);
};

#endif