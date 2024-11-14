#ifndef _SPIRAL_PATH_H
#define _SPIRAL_PATH_H

#include <ros/ros.h>

#include <algorithm>
#include <cppad/ipopt/solve.hpp>
#include <vector>

#include "../../../ThirdParty/UniROS.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "spiral_planner/circle.h"
#include "spiral_planner/line.h"
#include "spiral_planner/spiral.h"

using CppAD::AD;

struct sPosition
{
    double m_dX;
    double m_dY;
    double m_dYaw;

    sPosition() {}

    sPosition(double dX, double dY, double dYaw)
    {
        m_dX = dX;
        m_dY = dY;
        m_dYaw = dYaw;
    }
};

class CFG_eval
{
  public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    CFG_eval(){};
    CFG_eval(sPosition iStartPosition, sPosition iGoalPosition);
    std::vector<AD<double>> mapping_k2a(const AD<double>& k0, const AD<double>& k1, const AD<double>& k2,
                                        const AD<double>& k3, const AD<double>& s);
    AD<double> theta(const AD<double> theta_0, const AD<double>& a0, const AD<double>& a1, const AD<double>& a2,
                     const AD<double>& a3, const AD<double>& s);
    AD<double> curv(const AD<double>& a0, const AD<double>& a1, const AD<double>& a2, const AD<double>& a3,
                    const AD<double>& s);
    void setStartPosition(const sPosition iStartPosition) { this->m_iStartPosition = iStartPosition; };
    void setGoalPosition(const sPosition iGoalPosition) { this->m_iGoalPosition = iGoalPosition; };
    void setPlanner(const std::string strPlanner) { this->m_strPlanner = strPlanner; };
    void operator()(ADvector& fg, const ADvector& p);

  private:
    sPosition m_iStartPosition;
    sPosition m_iGoalPosition;
    std::string m_strPlanner;

    double m_dMin_curv_r = 2.5;
    double m_dMax_curv = 1 / m_dMin_curv_r;

    AD<double> position_x(const AD<double> position_x0, const AD<double> theta_0, const AD<double>& a0,
                          const AD<double>& a1, const AD<double>& a2, const AD<double>& a3, const AD<double>& s);
    AD<double> position_y(const AD<double> position_y0, const AD<double> theta_0, const AD<double>& a0,
                          const AD<double>& a1, const AD<double>& a2, const AD<double>& a3, const AD<double>& s);
    AD<double> cost(const AD<double>& a0, const AD<double>& a1, const AD<double>& a2, const AD<double>& a3,
                    const AD<double>& s);
};

class CShort_Distance_Planner
{
  public:
    typedef CPPAD_TESTVECTOR(double) Dvector;
    CShort_Distance_Planner();
    ~CShort_Distance_Planner();
    CFG_eval* m_ptrFG_eval;

  private:
    // ROS related
    ros::NodeHandle m_nh;
    ros::ServiceServer m_servSpiralPlanner;
    ros::ServiceServer m_servCirclePlanner;
    ros::ServiceServer m_servLinePlanner;

    double m_dS_cost_coef;
    double m_Goal_cost_tolerence;
    double m_dCircle_path_r_max = 5.0;
    double m_dCircle_path_r_min = 3.0;
    std::string m_strPlanner;

    // circle path plan
    int circle_origin_flag(sPosition iStartPosition, sPosition iGoalPosition);
    sPosition circle_origin_finder(sPosition iStartPosition, double r, int flag);
    std::vector<sPosition> circle_path_finder(sPosition iStartPosition, sPosition origin_position, double yaw_inter,
                                              int flag);

    // line path plan
    std::vector<sPosition> line_path_finder(sPosition start_positio, int direction, double distance);

    // spiral path plan
    std::vector<double> cumulative_trapezoid(const std::vector<double>& vecDiscrete_function_value,
                                             const std::vector<double>& vecX_set, const double dInitial);
    void spiral_path_finder(sPosition iStartPosition, sPosition iGoalPosition, std::vector<double>& vecSpiral_path_x,
                            std::vector<double>& vecSpiral_path_y, std::vector<double>& vecSpiral_path_yaw,
                            std::vector<double>& vecSpiral_path_curv, double& dMin_curvr, bool& bSFlag, double& dS);

    bool spiral_plan_srv_callback(spiral_planner::spiral::Request& request, spiral_planner::spiral::Response& response);
    bool line_plan_srv_callback(spiral_planner::line::Request& request, spiral_planner::line::Response& response);
    bool circle_plan_srv_callback(spiral_planner::circle::Request& request, spiral_planner::circle::Response& response);
    void printSolStatus(const CppAD::ipopt::solve_result<Dvector>::status_type& enumStatusType);
    void setSolverOptions(std::string& strOptions);
    void CalcDiscretePath(const sPosition& iStartPosition, const double dS, const std::vector<AD<double>>& res_a,
                          std::vector<double>& vecSpiral_path_x, std::vector<double>& vecSpiral_path_y,
                          std::vector<double>& vecSpiral_path_yaw, std::vector<double>& vecSpiral_path_curv,
                          double& dMin_curvr);
    void CalcSolFlag(const sPosition& iStartPosition, const sPosition& iGoalPosition, const double& dS,
                     const double& dSpiral_path_last_x, const double& dSpiral_path_last_y,
                     const double& dSpiral_path_last_yaw, bool& bSFlag);
};

#endif