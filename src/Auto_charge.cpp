#include "Auto_charge.h"

//********* callback function **************//

bool CAuto_Charge_Planner::spiral_plan_srv_callback(spiral_planner::spiral::Request& request,
                                                    spiral_planner::spiral::Response& response)
{
    MINF("================================");  // LOG divider
    m_uCurrent_iter = 0;
    m_bSolStatus = false;
    geometry_msgs::Point p1 = request.start_position;
    geometry_msgs::Point p2 = request.goal_position;

    sPosition iStartPosition(p1.x, p1.y, p1.z);
    sPosition iGoalPosition(p2.x, p2.y, p2.z);
    MINF("Start position:(%6.3f,%6.3f,%6.3f)", iStartPosition.m_dX, iStartPosition.m_dY, iStartPosition.m_dYaw);
    MINF("Goal position:(%6.3f,%6.3f,%6.3f)", iGoalPosition.m_dX, iGoalPosition.m_dY, iGoalPosition.m_dYaw);
    sPosition iGoalPosition_1, iGoalPosition_2;
    if (iStartPosition.m_dYaw < 0)
    {
        iGoalPosition_1 = sPosition(10, 4.85, -M_PI);
        iGoalPosition_2 = sPosition(16, 4.85, -M_PI);
    }
    else
    {
        iGoalPosition_1 = sPosition(10, 4.85, M_PI);
        iGoalPosition_2 = sPosition(16, 4.85, M_PI);
    }

    m_ptrFG_eval->reset();
    m_ptrFG_eval->setStartPosition(iStartPosition);
    m_ptrFG_eval->setGoalPosition(iGoalPosition_1);
    std::vector<double> vSpiral_path_x_1, vSpiral_path_y_1, vSpiral_path_yaw_1, vSpiral_path_curv_1;
    double dMin_curvr_1, dS_1, dCost_1;
    bool bSFlag_1;
    ros::Time Current = ros::Time::now();
    spiral_path_finder(iStartPosition, iGoalPosition_1, vSpiral_path_x_1, vSpiral_path_y_1, vSpiral_path_yaw_1,
                       vSpiral_path_curv_1, dMin_curvr_1, bSFlag_1, dS_1, dCost_1);
    double dMin_Y_1 = find_min_y(vSpiral_path_x_1, vSpiral_path_y_1, vSpiral_path_yaw_1);

    m_ptrFG_eval->reset();
    m_ptrFG_eval->setStartPosition(iStartPosition);
    m_ptrFG_eval->setGoalPosition(iGoalPosition_2);
    std::vector<double> vSpiral_path_x_2, vSpiral_path_y_2, vSpiral_path_yaw_2, vSpiral_path_curv_2;
    double dMin_curvr_2, dS_2, dCost_2;
    bool bSFlag_2;
    spiral_path_finder(iStartPosition, iGoalPosition_2, vSpiral_path_x_2, vSpiral_path_y_2, vSpiral_path_yaw_2,
                       vSpiral_path_curv_2, dMin_curvr_2, bSFlag_2, dS_2, dCost_2);
    double dMin_Y_2 = find_min_y(vSpiral_path_x_2, vSpiral_path_y_2, vSpiral_path_yaw_2);

    std::vector<double> vSpiral_path_x, vSpiral_path_y, vSpiral_path_yaw, vSpiral_path_curv;
    double dMin_curvr, dS;
    bool bSFlag;
    if (fabs(dMin_Y_1 - dMin_Y_2) >= 0.03)
    {
        if (dMin_Y_1 > dMin_Y_2)
        {
            vSpiral_path_x = vSpiral_path_x_1;
            vSpiral_path_y = vSpiral_path_y_1;
            vSpiral_path_yaw = vSpiral_path_yaw_1;
            vSpiral_path_curv = vSpiral_path_curv_1;
            dMin_curvr = dMin_curvr_1;
            dS = dS_1;
            bSFlag = bSFlag_1;
        }
        else
        {
            vSpiral_path_x = vSpiral_path_x_2;
            vSpiral_path_y = vSpiral_path_y_2;
            vSpiral_path_yaw = vSpiral_path_yaw_2;
            vSpiral_path_curv = vSpiral_path_curv_2;
            dMin_curvr = dMin_curvr_2;
            dS = dS_2;
            bSFlag = bSFlag_2;
        }
    }
    else if (dCost_1 <= dCost_2)
    {
        vSpiral_path_x = vSpiral_path_x_1;
        vSpiral_path_y = vSpiral_path_y_1;
        vSpiral_path_yaw = vSpiral_path_yaw_1;
        vSpiral_path_curv = vSpiral_path_curv_1;
        dMin_curvr = dMin_curvr_1;
        dS = dS_1;
        bSFlag = bSFlag_1;
    }
    else
    {
        vSpiral_path_x = vSpiral_path_x_2;
        vSpiral_path_y = vSpiral_path_y_2;
        vSpiral_path_yaw = vSpiral_path_yaw_2;
        vSpiral_path_curv = vSpiral_path_curv_2;
        dMin_curvr = dMin_curvr_2;
        dS = dS_2;
        bSFlag = bSFlag_2;
    }

    MINF("AutoCharge path planning time: %6.3f ms", 1000 * (ros::Time::now() - Current).toSec());
    MINF("Min_curvr:%6.3f", dMin_curvr);
    MINF("Total distance:%6.3f", dS);
    MINF("Min_Y_front:%6.3f,Min_Y_back:%6.3f", dMin_Y_1, dMin_Y_2);

    geometry_msgs::PoseArray traj;
    for (muint i = 0; i < vSpiral_path_x.size(); i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = vSpiral_path_x[i];
        pose.position.y = vSpiral_path_y[i];
        pose.orientation.z = vSpiral_path_yaw[i];
        pose.orientation.w = vSpiral_path_curv[i];
        traj.poses.push_back(pose);
    }
    response.traj = traj;

    response.min_curvr = dMin_curvr;
    response.s_flag = bSFlag;

    return true;
}

double CAuto_Charge_Planner::find_min_y(const std::vector<double>& vSpiral_path_x,
                                        const std::vector<double>& vSpiral_path_y,
                                        const std::vector<double>& vSpiral_path_yaw)
{
    double dMinY = 1e7;
    for (muint i = 0; i < vSpiral_path_x.size(); ++i)
    {
        double dForkliftX = vSpiral_path_x[i];
        double dForkliftY = vSpiral_path_y[i];
        double dForkliftYaw = vSpiral_path_yaw[i];
        Eigen::Matrix3d matRotation;
        matRotation << cos(dForkliftYaw), -sin(dForkliftYaw), dForkliftX, sin(dForkliftYaw), cos(dForkliftYaw),
            dForkliftY, 0, 0, 1;
        Eigen::MatrixXd matPosition_offset(3, 4);
        matPosition_offset << m_dForklift_x_front, m_dForklift_x_front, m_dForklift_x_back, m_dForklift_x_back,
            m_dForklift_y_left, m_dForklift_y_right, m_dForklift_y_left, m_dForklift_y_right, 1, 1, 1, 1;
        Eigen::MatrixXd matGlobalPos = matRotation * matPosition_offset;
        double dCurrent_min_y = matGlobalPos.row(1).minCoeff();
        dMinY = dCurrent_min_y < dMinY ? dCurrent_min_y : dMinY;
    }
    return dMinY;
}