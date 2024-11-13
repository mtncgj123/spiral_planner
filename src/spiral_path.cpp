#include "Spiral_Path.h"

using CppAD::AD;

// CFG_eval function definition

CFG_eval::CFG_eval(sPosition iStartPosition, sPosition iGoalPosition)
{
    m_iStartPosition = iStartPosition;
    m_iGoalPosition = iGoalPosition;
}

AD<double> CFG_eval::curv(const AD<double>& a0, const AD<double>& a1, const AD<double>& a2, const AD<double>& a3,
                          const AD<double>& s)
{
    return a3 * pow(s, 3) + a2 * pow(s, 2) + a1 * s + a0;
}

AD<double> CFG_eval::theta(const AD<double> theta_0, const AD<double>& a0, const AD<double>& a1, const AD<double>& a2,
                           const AD<double>& a3, const AD<double>& s)
{
    return theta_0 + a3 * pow(s, 4) / 4.0 + a2 * pow(s, 3) / 3.0 + a1 * pow(s, 2) / 2.0 + a0 * s;
}

AD<double> CFG_eval::position_x(const AD<double> position_x0, const AD<double> theta_0, const AD<double>& a0,
                                const AD<double>& a1, const AD<double>& a2, const AD<double>& a3, const AD<double>& s)
{
    return position_x0 +
           s / 24.0 *
               (cos(theta(theta_0, a0, a1, a2, a3, 0)) + 4 * cos(theta(theta_0, a0, a1, a2, a3, s / 8.0)) +
                2 * cos(theta(theta_0, a0, a1, a2, a3, s * 2 / 8.0)) +
                4 * cos(theta(theta_0, a0, a1, a2, a3, s * 3 / 8.0)) +
                2 * cos(theta(theta_0, a0, a1, a2, a3, s * 4 / 8.0)) +
                4 * cos(theta(theta_0, a0, a1, a2, a3, s * 5 / 8.0)) +
                2 * cos(theta(theta_0, a0, a1, a2, a3, s * 6 / 8.0)) +
                4 * cos(theta(theta_0, a0, a1, a2, a3, s * 7 / 8.0)) +
                cos(theta(theta_0, a0, a1, a2, a3, s * 8 / 8.0)));
}

AD<double> CFG_eval::position_y(const AD<double> position_y0, const AD<double> theta_0, const AD<double>& a0,
                                const AD<double>& a1, const AD<double>& a2, const AD<double>& a3, const AD<double>& s)
{
    return position_y0 +
           s / 24.0 *
               (sin(theta(theta_0, a0, a1, a2, a3, 0)) + 4 * sin(theta(theta_0, a0, a1, a2, a3, s / 8.0)) +
                2 * sin(theta(theta_0, a0, a1, a2, a3, s * 2 / 8.0)) +
                4 * sin(theta(theta_0, a0, a1, a2, a3, s * 3 / 8.0)) +
                2 * sin(theta(theta_0, a0, a1, a2, a3, s * 4 / 8.0)) +
                4 * sin(theta(theta_0, a0, a1, a2, a3, s * 5 / 8.0)) +
                2 * sin(theta(theta_0, a0, a1, a2, a3, s * 6 / 8.0)) +
                4 * sin(theta(theta_0, a0, a1, a2, a3, s * 7 / 8.0)) +
                sin(theta(theta_0, a0, a1, a2, a3, s * 8 / 8.0)));
}

AD<double> CFG_eval::cost(const AD<double>& a0, const AD<double>& a1, const AD<double>& a2, const AD<double>& a3,
                          const AD<double>& s)
{
    // double x_init = start_position.x;
    // double y_init = start_position.y;
    // double yaw_init = start_position.yaw;

    // double x_goal = m_iGoalPosition.x;
    // double y_goal = m_iGoalPosition.y;
    // double yaw_goal = m_iGoalPosition.yaw;

    // return (pow(x23, 2) / 7 * pow(x24, 7) +
    //         2 / 6 * x23 * x22 * pow(x24, 6) +
    //         (pow(x22, 2) + 2 * x23 * x21 + 9 * pow(x23, 2)) / 5 * pow(x24, 5) +
    //         (2 * x23 * x20 + 2 * x22 * x21 + 12 * x22 * x23) / 4 * pow(x24, 4) +
    //         (pow(x21, 2) + 2 * x22 * x20 + 4 * pow(x22, 2) + 6 * x21 * x23) / 3 * pow(x24, 3) +
    //         (2 * x21 * x20 + 4 * x21 * x22) / 2 * pow(x24, 2) +
    //         (pow(x20, 2) + pow(x21, 2)) * x24)+
    //         (pow(x13, 2) / 7 * pow(x14, 7) +
    //         2 / 6 * x13 * x12 * pow(x14, 6) +
    //         (pow(x12, 2) + 2 * x13 * x11 + 9 * pow(x13, 2)) / 5 * pow(x14, 5) +
    //         (2 * x13 * x10 + 2 * x12 * x11 + 12 * x12 * x13) / 4 * pow(x14, 4) +
    //         (pow(x11, 2) + 2 * x12 * x10 + 4 * pow(x12, 2) + 6 * x11 * x13) / 3 * pow(x14, 3) +
    //         (2 * x11 * x10 + 4 * x11 * x12) / 2 * pow(x14, 2) +
    //         (pow(x10, 2) + pow(x11, 2)) * x14)+
    //         0.5 * pow(x14, 2) + 0.5 * pow(x24, 2);

    // return (
    //         pow(x13, 2) / 7 * pow(x14, 7) + pow(x12, 2) / 5 * pow(x14, 5) + pow(x11, 2) / 3 * pow(x14, 3) + pow(x10,
    //         2) * x14 + 2 / 6 * x12 * x13 * pow(x14, 6) + 2 / 4 * x11 * x12 * pow(x14, 4) + 2 / 3 * x10 * x12 *
    //         pow(x14, 3) + 2 / 5 * x13 * x11 * pow(x14, 5) + 2 / 4 * x13 * x10 * pow(x14, 4) + x11 * x10 * pow(x14,
    //         2)) +
    //        1 * pow(x14, 2) + 1 * pow(x24, 2);

    return (pow(a3, 2) / 7.0 * pow(s, 7) + 2 / 6.0 * a3 * a2 * pow(s, 6) +
            (pow(a2, 2) + 2 * a3 * a1 + 9 * pow(a3, 2)) / 5.0 * pow(s, 5) +
            (2 * a3 * a0 + 2 * a2 * a1 + 12 * a2 * a3) / 4.0 * pow(s, 4) +
            (pow(a1, 2) + 2 * a2 * a0 + 4 * pow(a2, 2) + 6 * a1 * a3) / 3.0 * pow(s, 3) +
            (2 * a1 * a0 + 4 * a1 * a2) / 2.0 * pow(s, 2) + (pow(a0, 2) + pow(a1, 2)) * s);

    // 100 * pow(position_x(x_init, yaw_init, x0, x1, x2, x3, x4) - x_goal, 2) +
    // 100 * pow(position_y(y_init, yaw_init, x0, x1, x2, x3, x4) - y_goal, 2) +
    // 100 * pow(theta(yaw_init, x0, x1, x2, x3, x4) - yaw_goal, 2));
}

std::vector<AD<double>> CFG_eval::mapping_k2a(const AD<double>& k0, const AD<double>& k1, const AD<double>& k2,
                                              const AD<double>& k3, const AD<double>& s)
{
    std::vector<AD<double>> vecParam_a;  // vecParam_a={a0,a1,a2,a3,s}
    AD<double> a1 = -(11.0 / 2.0 * k0 - 9 * k1 + 9 / 2.0 * k2 - k3) / s;
    AD<double> a2 = (9 * k0 - 45 / 2.0 * k1 + 18.0 * k2 - 9 / 2.0 * k3) / pow(s, 2);
    AD<double> a3 = -(9 / 2.0 * k0 - 27 / 2.0 * k1 + 27 / 2.0 * k2 - 9 / 2.0 * k3) / pow(s, 3);
    vecParam_a.emplace_back(k0);
    vecParam_a.emplace_back(a1);
    vecParam_a.emplace_back(a2);
    vecParam_a.emplace_back(a3);
    vecParam_a.emplace_back(s);
    return vecParam_a;
}

void CFG_eval::operator()(ADvector& fg, const ADvector& p)
{
    assert(fg.size() == 9);
    assert(p.size() == 5);
    // variables
    AD<double> k0 = p[0];
    AD<double> k1 = p[1];
    AD<double> k2 = p[2];
    AD<double> k3 = p[3];
    AD<double> s = p[4];

    // f(x) objective function
    std::vector<AD<double>> polyparam_set = mapping_k2a(k0, k1, k2, k3, s);
    fg[0] = cost(polyparam_set[0], polyparam_set[1], polyparam_set[2], polyparam_set[3], polyparam_set[4]);

    // constraints
    fg[1] = k0;
    fg[2] = pow(m_dMax_curv, 2) - pow(k1, 2);
    fg[3] = pow(m_dMax_curv, 2) - pow(k2, 2);
    fg[4] = pow(m_dMax_curv, 2) - pow(k3, 2);
    fg[5] = s;

    fg[6] = position_x(m_iStartPosition.m_dX, m_iStartPosition.m_dYaw, polyparam_set[0], polyparam_set[1],
                       polyparam_set[2], polyparam_set[3], polyparam_set[4]) -
            m_iGoalPosition.m_dX;
    fg[7] = position_y(m_iStartPosition.m_dY, m_iStartPosition.m_dYaw, polyparam_set[0], polyparam_set[1],
                       polyparam_set[2], polyparam_set[3], polyparam_set[4]) -
            m_iGoalPosition.m_dY;
    fg[8] = theta(m_iStartPosition.m_dYaw, polyparam_set[0], polyparam_set[1], polyparam_set[2], polyparam_set[3],
                  polyparam_set[4]) -
            m_iGoalPosition.m_dYaw;

    return;
}

// CShort_Distance_Planner function definition

CShort_Distance_Planner::CShort_Distance_Planner()
{
    m_servSpiralPlanner =
        m_nh.advertiseService("spiral_plan_request", &CShort_Distance_Planner::spiral_plan_srv_callback, this);
    m_servCirclePlanner =
        m_nh.advertiseService("circle_plan_request", &CShort_Distance_Planner::circle_plan_srv_callback, this);
    m_servLinePlanner =
        m_nh.advertiseService("line_plan_request", &CShort_Distance_Planner::line_plan_srv_callback, this);

    m_nh.param("S_cost_coef", m_dS_cost_coef, 2.0);
    m_nh.param("Goal_cost_tolerence", m_Goal_cost_tolerence, 0.1);

    this->m_ptrFG_eval = new CFG_eval();
}
CShort_Distance_Planner::~CShort_Distance_Planner() { delete m_ptrFG_eval; }

// circle path plan
int CShort_Distance_Planner::circle_origin_flag(sPosition iStartPosition, sPosition iGoalPosition)
{
    std::vector<double> start_dir = {cos(iStartPosition.m_dYaw), sin(iStartPosition.m_dYaw)};
    std::vector<double> goal_dir = {cos(iGoalPosition.m_dYaw), sin(iGoalPosition.m_dYaw)};

    if (start_dir[0] * goal_dir[1] - start_dir[1] * goal_dir[0] >= 0)
    {
        return -1;
    }
    return 1;
}

sPosition CShort_Distance_Planner::circle_origin_finder(sPosition iStartPosition, double r, int flag)
{
    double start_position_x = iStartPosition.m_dX;
    double start_position_y = iStartPosition.m_dY;
    double start_position_yaw = iStartPosition.m_dYaw;

    double theta_origin = start_position_yaw + M_PI / 2 * flag;
    double origin_x = start_position_x + r * cos(theta_origin);
    double origin_y = start_position_y + r * sin(theta_origin);
    sPosition origin = {origin_x, origin_y, 0};

    return origin;
}

std::vector<sPosition> CShort_Distance_Planner::circle_path_finder(sPosition iStartPosition, sPosition origin_position,
                                                                   double yaw_inter, int flag)
{
    double theta_ori2goal = yaw_inter - M_PI / 2 * flag;
    double rotation_direction;
    std::vector<sPosition> circle_path;

    double start_position_x = iStartPosition.m_dX;
    double start_position_y = iStartPosition.m_dY;
    double start_position_yaw = iStartPosition.m_dYaw;

    double origin_x = origin_position.m_dX;
    double origin_y = origin_position.m_dY;

    std::vector<double> dir_ori2start = {start_position_x - origin_x, start_position_y - origin_y};
    std::vector<double> dir_ori2goal = {cos(theta_ori2goal), sin(theta_ori2goal)};

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
    sPosition p;
    for (int i = 0; i < n_theta_step + 1; i++)
    {
        p.m_dX = origin_x +
                 cos(i / n_theta_step * theta_rotation * rotation_direction) * (start_position_x - origin_x) -
                 sin(i / n_theta_step * theta_rotation * rotation_direction) * (start_position_y - origin_y);

        p.m_dY = origin_y +
                 sin(i / n_theta_step * theta_rotation * rotation_direction) * (start_position_x - origin_x) +
                 cos(i / n_theta_step * theta_rotation * rotation_direction) * (start_position_y - origin_y);

        p.m_dYaw = start_position_yaw + i / n_theta_step * theta_rotation * rotation_direction;

        circle_path.push_back(p);
    }
    return circle_path;
}

// line path plan

std::vector<sPosition> CShort_Distance_Planner::line_path_finder(sPosition iStartPosition, int direction,
                                                                 double distance)
{
    double start_position_x = iStartPosition.m_dX;
    double start_position_y = iStartPosition.m_dY;
    double start_position_yaw = iStartPosition.m_dYaw;
    std::vector<sPosition> line_path;
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
        sPosition p;
        p.m_dX = start_position_x + i * path_step * cos(moving_yaw);
        p.m_dY = start_position_y + i * path_step * sin(moving_yaw);
        p.m_dYaw = start_position_yaw;
        line_path.push_back(p);
    }
    return line_path;
}

// spiral path plan

std::vector<double> CShort_Distance_Planner::cumulative_trapezoid(const std::vector<double>& vecDiscrete_function_value,
                                                                  const std::vector<double>& vecX_set,
                                                                  const double dInitial)
{
    std::vector<double> cumulative_res;
    double dSum = dInitial;
    cumulative_res.push_back(dSum);
    for (muint i = 0; i < vecX_set.size() - 1; i++)
    {
        dSum +=
            (vecDiscrete_function_value[i] + vecDiscrete_function_value[i + 1]) / 2.0 * (vecX_set[i + 1] - vecX_set[i]);
        cumulative_res.push_back(dSum);
    }
    return cumulative_res;
}

void CShort_Distance_Planner::spiral_path_finder(sPosition iStartPosition, sPosition iGoalPosition,
                                                 std::vector<double>& vecSpiral_path_x,
                                                 std::vector<double>& vecSpiral_path_y,
                                                 std::vector<double>& vecSpiral_path_yaw, double& dMin_curvr,
                                                 bool& bSFlag, double& dS)
{
    vecSpiral_path_x.clear();
    vecSpiral_path_y.clear();
    vecSpiral_path_yaw.clear();

    size_t i;

    size_t nx = 5;  // number of varibles
    size_t ng = 8;  // number of constraints
    Dvector v(nx);  // initial condition of varibles
    v[0] = 0.0;     // a0
    v[1] = 0.0;     // a1
    v[2] = 0.0;     // a2
    v[3] = 0.0;     // a3
    v[4] = 10.0;    // s

    // lower and upper bounds for varibles
    Dvector xl(nx), xu(nx);
    for (i = 0; i < nx; i++)
    {
        xl[i] = -100;
        xu[i] = +100;
    }

    Dvector gl(ng), gu(ng);
    gl[0] = 0;  // k0
    gu[0] = 0;
    gl[1] = 0;  // kmax^2-k1^2
    gu[1] = 1.0e19;
    gl[2] = 0;  // kmax^2-k2^2
    gu[2] = 1.0e19;
    gl[3] = 0;  // kmax^2-k3^2
    gu[3] = 1.0e19;
    gl[4] = -1.0e2;  // s
    gu[4] = 1.0e2;
    gl[5] = 0;  // position_x-goal_x
    gu[5] = 0;
    gl[6] = 0;  // position_y-goal_y
    gu[6] = 0;
    gl[7] = 0;  // position_yaw-goal_yaw
    gu[7] = 0;

    // options
    std::string options;
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
    CppAD::ipopt::solve<Dvector, CFG_eval>(options, v, xl, xu, gl, gu, *m_ptrFG_eval, solution);

    std::vector<AD<double>> res_x1 =
        m_ptrFG_eval->mapping_k2a((solution.x)[0], (solution.x)[1], (solution.x)[2], (solution.x)[3], (solution.x)[4]);

    dS = CppAD::Value(res_x1[4]);
    double dStep = 0.01;
    std::vector<double> x_discrete, y_discrete, spiral_path_curvr, s_set;
    if (dS > 0)
    {
        for (double i = 0; i < dS + dStep; i += dStep)
        {
            double theta_yaw =
                CppAD::Value(m_ptrFG_eval->theta(iStartPosition.m_dYaw, res_x1[0], res_x1[1], res_x1[2], res_x1[3], i));
            x_discrete.push_back(cos(theta_yaw));
            y_discrete.push_back(sin(theta_yaw));
            vecSpiral_path_yaw.push_back(theta_yaw);
            s_set.push_back(i);
            spiral_path_curvr.push_back(
                1.0 /
                std::max(fabs(CppAD::Value(m_ptrFG_eval->curv(res_x1[0], res_x1[1], res_x1[2], res_x1[3], i))), 0.01));
        }
    }
    else
    {
        for (double i = 0; i > dS - dStep; i -= dStep)
        {
            double theta_yaw =
                CppAD::Value(m_ptrFG_eval->theta(iStartPosition.m_dYaw, res_x1[0], res_x1[1], res_x1[2], res_x1[3], i));
            x_discrete.push_back(cos(theta_yaw));
            y_discrete.push_back(sin(theta_yaw));
            vecSpiral_path_yaw.push_back(theta_yaw);
            s_set.push_back(i);
            spiral_path_curvr.push_back(
                1.0 /
                std::max(fabs(CppAD::Value(m_ptrFG_eval->curv(res_x1[0], res_x1[1], res_x1[2], res_x1[3], i))), 0.01));
        }
    }
    vecSpiral_path_x = cumulative_trapezoid(x_discrete, s_set, iStartPosition.m_dX);
    vecSpiral_path_y = cumulative_trapezoid(y_discrete, s_set, iStartPosition.m_dY);
    dMin_curvr = *min_element(spiral_path_curvr.begin(), spiral_path_curvr.end());

    double dEuclideanDist =
        sqrt(pow(iStartPosition.m_dX - iGoalPosition.m_dX, 2) + pow(iStartPosition.m_dY - iGoalPosition.m_dY, 2));
    bool bSCost = dS < m_dS_cost_coef * dEuclideanDist;

    double dGoalError = sqrt(pow(vecSpiral_path_x.back() - iGoalPosition.m_dX, 2) +
                             pow(vecSpiral_path_y.back() - iGoalPosition.m_dY, 2) +
                             pow(vecSpiral_path_yaw.back() - iGoalPosition.m_dYaw, 2));
    bool bGoalCost = (dGoalError < m_Goal_cost_tolerence);

    if (false == bSCost)
    {
        MINF("S_cost false s:%6.3f,Euclidean dist cost:%6.3f", dS, m_dS_cost_coef * dEuclideanDist);
    }
    if (false == bGoalCost)
    {
        MINF("X_Error,Y_Error,Yaw_Error: (%6.3f, %6.3f, %6.3f)", fabs(vecSpiral_path_x.back() - iGoalPosition.m_dX),
             fabs(vecSpiral_path_y.back() - iGoalPosition.m_dY),
             fabs(vecSpiral_path_yaw.back() - iGoalPosition.m_dYaw));
    }

    if (bSCost == true && bGoalCost == true)
    {
        bSFlag = true;
    }
    else
    {
        bSFlag = false;
    }
}

//********* callback function **************//

bool CShort_Distance_Planner::spiral_plan_srv_callback(spiral_planner::spiral::Request& request,
                                                       spiral_planner::spiral::Response& response)
{
    MINF("================================");  // LOG divider
    geometry_msgs::Point p1 = request.start_position;
    geometry_msgs::Point p2 = request.goal_position;

    sPosition iStartPosition(p1.x, p1.y, p1.z);
    sPosition iGoalPosition(p2.x, p2.y, p2.z);
    MINF("Start position:(%6.3f,%6.3f,%6.3f)", iStartPosition.m_dX, iStartPosition.m_dY, iStartPosition.m_dYaw);
    MINF("Goal position:(%6.3f,%6.3f,%6.3f)", iGoalPosition.m_dX, iGoalPosition.m_dY, iGoalPosition.m_dYaw);
    this->m_ptrFG_eval->setStartPosition(iStartPosition);
    this->m_ptrFG_eval->setGoalPosition(iGoalPosition);
    std::vector<double> vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw;
    double dMin_curvr, dS;
    bool bSFlag;

    ros::Time Current = ros::Time::now();
    spiral_path_finder(iStartPosition, iGoalPosition, vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw,
                       dMin_curvr, bSFlag, dS);
    MINF("Spiral path planning time: %6.3f ms", 1000 * (ros::Time::now() - Current).toSec());
    MINF("Min_curvr:%6.3f", dMin_curvr);
    MINF("Total distance:%6.3f", dS);

    geometry_msgs::PoseArray traj;
    for (muint i = 0; i < vecSpiral_path_x.size(); i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = vecSpiral_path_x[i];
        pose.position.y = vecSpiral_path_y[i];
        pose.position.z = vecSpiral_path_yaw[i];
        traj.poses.push_back(pose);
    }
    response.traj = traj;

    response.min_curvr = dMin_curvr;
    response.s_flag = bSFlag;

    return true;
}

bool CShort_Distance_Planner::circle_plan_srv_callback(spiral_planner::circle::Request& request,
                                                       spiral_planner::circle::Response& response)
{
    geometry_msgs::Point p1 = request.start_position;
    geometry_msgs::Point p2 = request.goal_position;
    double yaw_inter = request.yaw_inter;
    double r = request.r;

    sPosition iStartPosition(p1.x, p1.y, p1.z);
    sPosition iGoalPosition(p2.x, p2.y, p2.z);
    int flag = this->circle_origin_flag(iStartPosition, iGoalPosition);
    sPosition _origin_position = this->circle_origin_finder(iStartPosition, r, flag);

    std::vector<sPosition> circle_path = this->circle_path_finder(iStartPosition, _origin_position, yaw_inter, flag);
    geometry_msgs::PoseArray traj;
    for (muint i = 0; i < circle_path.size(); i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = circle_path[i].m_dX;
        pose.position.y = circle_path[i].m_dY;
        pose.position.z = circle_path[i].m_dYaw;
        traj.poses.push_back(pose);
    }
    response.traj = traj;

    return true;
}

bool CShort_Distance_Planner::line_plan_srv_callback(spiral_planner::line::Request& request,
                                                     spiral_planner::line::Response& response)
{
    geometry_msgs::Point p1 = request.start_position;
    int direction = request.direction;
    double distance = request.distance;

    sPosition iStartPosition(p1.x, p1.y, p1.z);

    std::vector<sPosition> line_path = this->line_path_finder(iStartPosition, direction, distance);
    geometry_msgs::PoseArray traj;
    for (muint i = 0; i < line_path.size(); i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = line_path[i].m_dX;
        pose.position.y = line_path[i].m_dY;
        pose.position.z = line_path[i].m_dYaw;
        traj.poses.push_back(pose);
    }
    response.traj = traj;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spiral_planner");
    CShort_Distance_Planner iSDP;

    ros::spin();

    return 0;
}
