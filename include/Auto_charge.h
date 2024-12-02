#ifndef _AUTO_CHARGE_H
#define _AUTO_CHARGE_H

#include <eigen3/Eigen/Eigen>

#include "Spiral_Path.h"

class CAuto_Charge_Planner : public CShort_Distance_Planner
{
  public:
  private:
    double m_dForklift_x_front = 0.56;
    double m_dForklift_x_back = -2.48;
    double m_dForklift_y_left = 0.8;
    double m_dForklift_y_right = -0.8;
    bool spiral_plan_srv_callback(spiral_planner::spiral::Request& request,
                                  spiral_planner::spiral::Response& response) override;
    double find_min_y(const std::vector<double>& vSpiral_path_x, const std::vector<double>& vSpiral_path_y,
                      const std::vector<double>& vSpiral_path_yaw);
};

#endif