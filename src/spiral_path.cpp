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
    double dX_init = m_iStartPosition.m_dX;
    double dY_init = m_iStartPosition.m_dY;
    double dYaw_init = m_iStartPosition.m_dYaw;

    double dX_goal = m_iGoalPosition.m_dX;
    double dY_goal = m_iGoalPosition.m_dY;
    double dYaw_goal = m_iGoalPosition.m_dYaw;

    double dSoft_constraints_coef = 1000.0;
    double dS_cost_coef = 0.01;

    if ("origin" == m_strCost)
    {
        return (pow(a3, 2) / 7.0 * pow(s, 7) + 2 / 6.0 * a3 * a2 * pow(s, 6) +
                (pow(a2, 2) + 2 * a3 * a1 + 9 * pow(a3, 2)) / 5.0 * pow(s, 5) +
                (2 * a3 * a0 + 2 * a2 * a1 + 12 * a2 * a3) / 4.0 * pow(s, 4) +
                (pow(a1, 2) + 2 * a2 * a0 + 4 * pow(a2, 2) + 6 * a1 * a3) / 3.0 * pow(s, 3) +
                (2 * a1 * a0 + 4 * a1 * a2) / 2.0 * pow(s, 2) + (pow(a0, 2) + pow(a1, 2)) * s) *
               s / fabs(s);
    }
    else if ("soft" == m_strCost)
    {
        return (pow(a3, 2) / 7.0 * pow(s, 7) + 2 / 6.0 * a3 * a2 * pow(s, 6) +
                (pow(a2, 2) + 2 * a3 * a1 + 9 * pow(a3, 2)) / 5.0 * pow(s, 5) +
                (2 * a3 * a0 + 2 * a2 * a1 + 12 * a2 * a3) / 4.0 * pow(s, 4) +
                (pow(a1, 2) + 2 * a2 * a0 + 4 * pow(a2, 2) + 6 * a1 * a3) / 3.0 * pow(s, 3) +
                (2 * a1 * a0 + 4 * a1 * a2) / 2.0 * pow(s, 2) + (pow(a0, 2) + pow(a1, 2)) * s) *
                   s / fabs(s) +
               dSoft_constraints_coef * pow(position_x(dX_init, dYaw_init, a0, a1, a2, a3, s) - dX_goal, 2) +
               dSoft_constraints_coef * pow(position_y(dY_init, dYaw_init, a0, a1, a2, a3, s) - dY_goal, 2) +
               dSoft_constraints_coef * pow(theta(dYaw_init, a0, a1, a2, a3, s) - dYaw_goal, 2);
    }
    else if ("origin_with_s" == m_strCost)
    {
        return (pow(a3, 2) / 7.0 * pow(s, 7) + 2 / 6.0 * a3 * a2 * pow(s, 6) +
                (pow(a2, 2) + 2 * a3 * a1 + 9 * pow(a3, 2)) / 5.0 * pow(s, 5) +
                (2 * a3 * a0 + 2 * a2 * a1 + 12 * a2 * a3) / 4.0 * pow(s, 4) +
                (pow(a1, 2) + 2 * a2 * a0 + 4 * pow(a2, 2) + 6 * a1 * a3) / 3.0 * pow(s, 3) +
                (2 * a1 * a0 + 4 * a1 * a2) / 2.0 * pow(s, 2) + (pow(a0, 2) + pow(a1, 2)) * s) *
                   s / fabs(s) +
               dS_cost_coef * s * s;
    }
    else
    {
        MERR("Cost type invalid");
        return s;
    }
}

std::vector<AD<double>> CFG_eval::mapping_k2a(const AD<double>& k0, const AD<double>& k1, const AD<double>& k2,
                                              const AD<double>& k3, const AD<double>& s)
{
    AD<double> a1 = -(11.0 / 2.0 * k0 - 9 * k1 + 9 / 2.0 * k2 - k3) / s;
    AD<double> a2 = (9 * k0 - 45 / 2.0 * k1 + 18.0 * k2 - 9 / 2.0 * k3) / pow(s, 2);
    AD<double> a3 = -(9 / 2.0 * k0 - 27 / 2.0 * k1 + 27 / 2.0 * k2 - 9 / 2.0 * k3) / pow(s, 3);
    std::vector<AD<double>> vParam_a = {k0, a1, a2, a3, s};  // vParam_a={a0,a1,a2,a3,s}
    return vParam_a;
}

void CFG_eval::checkCurv(const std::vector<AD<double>>& res_a, const double& dMaxCurv, std::vector<double>& vInvalidS)
{
    double dParam_a1 = CppAD::Value(res_a[1]);
    double dParam_a2 = CppAD::Value(res_a[2]);
    double dParam_a3 = CppAD::Value(res_a[3]);
    double dS = CppAD::Value(res_a[4]);
    double dDelta = 4 * (dParam_a2 * dParam_a2 - 3 * dParam_a1 * dParam_a3);
    if (dDelta < 0)
    {
        return;
    }
    else if (0 == dDelta)
    {
        double dSol_s = -dParam_a2 / dParam_a3 / 3.0;
        double dCurv = CppAD::Value(curv(res_a[0], res_a[1], res_a[2], res_a[3], dSol_s));
        if (-dMaxCurv <= dCurv && dCurv <= dMaxCurv)
        {
            return;
        }
        else if (0 <= dSol_s / dS && dSol_s / dS <= 1)
        {
            vInvalidS.push_back(dSol_s);
            return;
        }
    }
    else
    {
        double dSol_s1 = (-2 * dParam_a2 + sqrt(dDelta)) / dParam_a3 / 6.0;
        double dSol_s2 = (-2 * dParam_a2 - sqrt(dDelta)) / dParam_a3 / 6.0;
        double dCurv1 = CppAD::Value(curv(res_a[0], res_a[1], res_a[2], res_a[3], dSol_s1));
        double dCurv2 = CppAD::Value(curv(res_a[0], res_a[1], res_a[2], res_a[3], dSol_s2));
        if (dCurv1 < -dMaxCurv || dCurv1 > dMaxCurv)
        {
            if (0 <= dSol_s1 / dS && dSol_s1 / dS <= 1)
            {
                vInvalidS.push_back(dSol_s1);
            }
        }
        if (dCurv2 < -dMaxCurv || dCurv2 > dMaxCurv)
        {
            if (0 <= dSol_s2 / dS && dSol_s2 / dS <= 1)
            {
                vInvalidS.push_back(dSol_s2);
                return;
            }
        }
        return;
    }
}

void CFG_eval::operator()(ADvector& fg, const ADvector& p)
{
    if ("origin" == m_strPlanner)
    {
        assert(p.size() == 5);
        assert(fg.size() == 9);
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
        fg[2] = k1;
        fg[3] = k2;
        fg[4] = k3;
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
    else if ("soft" == m_strPlanner)
    {
        assert(p.size() == 5);
        assert(fg.size() == 6);
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
        fg[2] = k1;
        fg[3] = k2;
        fg[4] = k3;
        fg[5] = s;

        return;
    }
    else if ("dual_path" == m_strPlanner)
    {
        assert(p.size() == 13);
        assert(fg.size() == 21);
        // variables
        AD<double> k_front0 = p[0];
        AD<double> k_front1 = p[1];
        AD<double> k_front2 = p[2];
        AD<double> k_front3 = p[3];
        AD<double> s_front = p[4];

        AD<double> k_back0 = p[5];
        AD<double> k_back1 = p[6];
        AD<double> k_back2 = p[7];
        AD<double> k_back3 = p[8];
        AD<double> s_back = p[9];

        AD<double> middle_x = p[10];
        AD<double> middle_y = p[11];
        AD<double> middle_yaw = p[12];

        // f(x) objective function
        std::vector<AD<double>> polyparam_set_front = mapping_k2a(k_front0, k_front1, k_front2, k_front3, s_front);
        std::vector<AD<double>> polyparam_set_back = mapping_k2a(k_back0, k_back1, k_back2, k_back3, s_back);
        fg[0] = cost(polyparam_set_front[0], polyparam_set_front[1], polyparam_set_front[2], polyparam_set_front[3],
                     polyparam_set_front[4]) +
                cost(polyparam_set_back[0], polyparam_set_back[1], polyparam_set_back[2], polyparam_set_back[3],
                     polyparam_set_back[4]);

        // constraints
        fg[1] = k_front0;
        fg[2] = k_front1;
        fg[3] = k_front2;
        fg[4] = k_front3;
        fg[5] = s_front;

        fg[6] =
            position_x(m_iStartPosition.m_dX, m_iStartPosition.m_dYaw, polyparam_set_front[0], polyparam_set_front[1],
                       polyparam_set_front[2], polyparam_set_front[3], polyparam_set_front[4]) -
            middle_x;
        fg[7] =
            position_y(m_iStartPosition.m_dY, m_iStartPosition.m_dYaw, polyparam_set_front[0], polyparam_set_front[1],
                       polyparam_set_front[2], polyparam_set_front[3], polyparam_set_front[4]) -
            middle_y;
        fg[8] = theta(m_iStartPosition.m_dYaw, polyparam_set_front[0], polyparam_set_front[1], polyparam_set_front[2],
                      polyparam_set_front[3], polyparam_set_front[4]) -
                middle_yaw;

        fg[9] = k_back0;
        fg[10] = k_back1;
        fg[11] = k_back2;
        fg[12] = k_back3;
        fg[13] = s_back;

        fg[14] = position_x(m_iGoalPosition.m_dX, m_iGoalPosition.m_dYaw, polyparam_set_back[0], polyparam_set_back[1],
                            polyparam_set_back[2], polyparam_set_back[3], polyparam_set_back[4]) -
                 middle_x;
        fg[15] = position_y(m_iGoalPosition.m_dY, m_iGoalPosition.m_dYaw, polyparam_set_back[0], polyparam_set_back[1],
                            polyparam_set_back[2], polyparam_set_back[3], polyparam_set_back[4]) -
                 middle_y;
        fg[16] = theta(m_iGoalPosition.m_dYaw, polyparam_set_back[0], polyparam_set_back[1], polyparam_set_back[2],
                       polyparam_set_back[3], polyparam_set_back[4]) -
                 middle_yaw;

        fg[17] = middle_x;
        fg[18] = middle_y;
        fg[19] = middle_yaw;
        fg[20] = k_back3 - k_front3;

        return;
    }
    else if ("dual_path_with_limit_check" == m_strPlanner)
    {
        muint uSize_vInvalidS_front = m_vInvalidS_front.size();
        muint uSize_vInvalidS_back = m_vInvalidS_back.size();
        MERR("CFG_eval invalidS size:%lld", uSize_vInvalidS_front + uSize_vInvalidS_back);
        assert(p.size() == 13);
        assert(fg.size() == 21 + uSize_vInvalidS_front + uSize_vInvalidS_back);
        // variables
        AD<double> k_front0 = p[0];
        AD<double> k_front1 = p[1];
        AD<double> k_front2 = p[2];
        AD<double> k_front3 = p[3];
        AD<double> s_front = p[4];

        AD<double> k_back0 = p[5];
        AD<double> k_back1 = p[6];
        AD<double> k_back2 = p[7];
        AD<double> k_back3 = p[8];
        AD<double> s_back = p[9];

        AD<double> middle_x = p[10];
        AD<double> middle_y = p[11];
        AD<double> middle_yaw = p[12];

        // f(x) objective function
        std::vector<AD<double>> polyparam_set_front = mapping_k2a(k_front0, k_front1, k_front2, k_front3, s_front);
        std::vector<AD<double>> polyparam_set_back = mapping_k2a(k_back0, k_back1, k_back2, k_back3, s_back);
        fg[0] = cost(polyparam_set_front[0], polyparam_set_front[1], polyparam_set_front[2], polyparam_set_front[3],
                     polyparam_set_front[4]) +
                cost(polyparam_set_back[0], polyparam_set_back[1], polyparam_set_back[2], polyparam_set_back[3],
                     polyparam_set_back[4]);

        // constraints
        fg[1] = k_front0;
        fg[2] = k_front1;
        fg[3] = k_front2;
        fg[4] = k_front3;
        fg[5] = s_front;

        fg[6] =
            position_x(m_iStartPosition.m_dX, m_iStartPosition.m_dYaw, polyparam_set_front[0], polyparam_set_front[1],
                       polyparam_set_front[2], polyparam_set_front[3], polyparam_set_front[4]) -
            middle_x;
        fg[7] =
            position_y(m_iStartPosition.m_dY, m_iStartPosition.m_dYaw, polyparam_set_front[0], polyparam_set_front[1],
                       polyparam_set_front[2], polyparam_set_front[3], polyparam_set_front[4]) -
            middle_y;
        fg[8] = theta(m_iStartPosition.m_dYaw, polyparam_set_front[0], polyparam_set_front[1], polyparam_set_front[2],
                      polyparam_set_front[3], polyparam_set_front[4]) -
                middle_yaw;

        fg[9] = k_back0;
        fg[10] = k_back1;
        fg[11] = k_back2;
        fg[12] = k_back3;
        fg[13] = s_back;

        fg[14] = position_x(m_iGoalPosition.m_dX, m_iGoalPosition.m_dYaw, polyparam_set_back[0], polyparam_set_back[1],
                            polyparam_set_back[2], polyparam_set_back[3], polyparam_set_back[4]) -
                 middle_x;
        fg[15] = position_y(m_iGoalPosition.m_dY, m_iGoalPosition.m_dYaw, polyparam_set_back[0], polyparam_set_back[1],
                            polyparam_set_back[2], polyparam_set_back[3], polyparam_set_back[4]) -
                 middle_y;
        fg[16] = theta(m_iGoalPosition.m_dYaw, polyparam_set_back[0], polyparam_set_back[1], polyparam_set_back[2],
                       polyparam_set_back[3], polyparam_set_back[4]) -
                 middle_yaw;

        fg[17] = middle_x;
        fg[18] = middle_y;
        fg[19] = middle_yaw;
        fg[20] = k_back3 - k_front3;

        for (muint i = 0; i < uSize_vInvalidS_front; ++i)
        {
            fg[21 + i] = curv(polyparam_set_front[0], polyparam_set_front[1], polyparam_set_front[2],
                              polyparam_set_front[3], m_vInvalidS_front[i]);
            MWARN("vInvalidS_fronti:%6.3f", m_vInvalidS_front[i]);
        }
        for (muint i = 0; i < uSize_vInvalidS_back; ++i)
        {
            fg[21 + uSize_vInvalidS_front + i] =
                curv(polyparam_set_back[0], polyparam_set_back[1], polyparam_set_back[2], polyparam_set_back[3],
                     m_vInvalidS_back[i]);
            MWARN("vInvalidS_backi:%6.3f", m_vInvalidS_back[i]);
        }

        return;
    }
    else if ("triple_path" == m_strPlanner)
    {
        assert(p.size() == 21);
        assert(fg.size() == 33);
        // variables
        AD<double> k_front0 = p[0];
        AD<double> k_front1 = p[1];
        AD<double> k_front2 = p[2];
        AD<double> k_front3 = p[3];
        AD<double> s_front = p[4];

        AD<double> k_inter0 = p[5];
        AD<double> k_inter1 = p[6];
        AD<double> k_inter2 = p[7];
        AD<double> k_inter3 = p[8];
        AD<double> s_inter = p[9];

        AD<double> k_back0 = p[10];
        AD<double> k_back1 = p[11];
        AD<double> k_back2 = p[12];
        AD<double> k_back3 = p[13];
        AD<double> s_back = p[14];

        AD<double> middle_x_front = p[15];
        AD<double> middle_y_front = p[16];
        AD<double> middle_yaw_front = p[17];

        AD<double> middle_x_back = p[18];
        AD<double> middle_y_back = p[19];
        AD<double> middle_yaw_back = p[20];

        std::vector<AD<double>> polyparam_set_front = mapping_k2a(k_front0, k_front1, k_front2, k_front3, s_front);
        std::vector<AD<double>> polyparam_set_inter = mapping_k2a(k_inter0, k_inter1, k_inter2, k_inter3, s_inter);
        std::vector<AD<double>> polyparam_set_back = mapping_k2a(k_back0, k_back1, k_back2, k_back3, s_back);
        fg[0] = cost(polyparam_set_front[0], polyparam_set_front[1], polyparam_set_front[2], polyparam_set_front[3],
                     polyparam_set_front[4]) +
                cost(polyparam_set_inter[0], polyparam_set_inter[1], polyparam_set_inter[2], polyparam_set_inter[3],
                     polyparam_set_inter[4]) +
                cost(polyparam_set_back[0], polyparam_set_back[1], polyparam_set_back[2], polyparam_set_back[3],
                     polyparam_set_back[4]);

        // constraints
        fg[1] = k_front0;
        fg[2] = k_front1;
        fg[3] = k_front2;
        fg[4] = k_front3;
        fg[5] = s_front;

        fg[6] =
            position_x(m_iStartPosition.m_dX, m_iStartPosition.m_dYaw, polyparam_set_front[0], polyparam_set_front[1],
                       polyparam_set_front[2], polyparam_set_front[3], polyparam_set_front[4]) -
            middle_x_front;
        fg[7] =
            position_y(m_iStartPosition.m_dY, m_iStartPosition.m_dYaw, polyparam_set_front[0], polyparam_set_front[1],
                       polyparam_set_front[2], polyparam_set_front[3], polyparam_set_front[4]) -
            middle_y_front;
        fg[8] = theta(m_iStartPosition.m_dYaw, polyparam_set_front[0], polyparam_set_front[1], polyparam_set_front[2],
                      polyparam_set_front[3], polyparam_set_front[4]) -
                middle_yaw_front;

        fg[9] = k_inter0;
        fg[10] = k_inter1;
        fg[11] = k_inter2;
        fg[12] = k_inter3;
        fg[13] = s_inter;

        fg[14] = position_x(middle_x_front, middle_yaw_front, polyparam_set_inter[0], polyparam_set_inter[1],
                            polyparam_set_inter[2], polyparam_set_inter[3], polyparam_set_inter[4]) -
                 middle_x_back;
        fg[15] = position_y(middle_y_front, middle_yaw_front, polyparam_set_inter[0], polyparam_set_inter[1],
                            polyparam_set_inter[2], polyparam_set_inter[3], polyparam_set_inter[4]) -
                 middle_y_back;
        fg[16] = theta(middle_yaw_front, polyparam_set_inter[0], polyparam_set_inter[1], polyparam_set_inter[2],
                       polyparam_set_inter[3], polyparam_set_inter[4]) -
                 middle_yaw_back;

        fg[17] = middle_x_front;
        fg[18] = middle_y_front;
        fg[19] = middle_yaw_front;
        fg[20] = middle_x_back;
        fg[21] = middle_y_back;
        fg[22] = middle_yaw_back;

        fg[23] = k_back0;
        fg[24] = k_back1;
        fg[25] = k_back2;
        fg[26] = k_back3;
        fg[27] = s_back;

        fg[28] = position_x(m_iGoalPosition.m_dX, m_iGoalPosition.m_dYaw, polyparam_set_back[0], polyparam_set_back[1],
                            polyparam_set_back[2], polyparam_set_back[3], polyparam_set_back[4]) -
                 middle_x_back;
        fg[29] = position_y(m_iGoalPosition.m_dY, m_iGoalPosition.m_dYaw, polyparam_set_back[0], polyparam_set_back[1],
                            polyparam_set_back[2], polyparam_set_back[3], polyparam_set_back[4]) -
                 middle_y_back;
        fg[30] = theta(m_iGoalPosition.m_dYaw, polyparam_set_back[0], polyparam_set_back[1], polyparam_set_back[2],
                       polyparam_set_back[3], polyparam_set_back[4]) -
                 middle_yaw_back;

        fg[31] = k_inter0 - k_front3;
        fg[32] = k_inter3 - k_back3;

        return;
    }
}

void CFG_eval::reset()
{
    setStartPosition(sPosition(0, 0, 0));
    setGoalPosition(sPosition(0, 0, 0));
    m_vInvalidS_front.clear();
    m_vInvalidS_back.clear();
}

// CShort_Distance_Planner function definition
CShort_Distance_Planner::CShort_Distance_Planner()
{
    m_servSpiralPlanner =
        m_nh.advertiseService("spiral_plan_request", &CShort_Distance_Planner::spiral_plan_srv_callback, this);

    m_nh.param("/spiral_planner/S_cost_coef", m_dS_cost_coef, 100.0);
    m_nh.param("/spiral_planner/Goal_cost_tolerence", m_Goal_cost_tolerence, 100.0);
    m_nh.param("/spiral_planner/planner", m_strPlanner, std::string("invalid"));
    m_nh.param("/spiral_planner/cost", m_strCost, std::string("invalid"));

    MINF("S_cost_coef %6.3f", m_dS_cost_coef);
    MINF("Goal_cost_tolerence %6.3f", m_Goal_cost_tolerence);
    MINF("m_strPlanner %s", m_strPlanner.c_str());
    MINF("m_strCost %s", m_strCost.c_str());

    m_ptrFG_eval = new CFG_eval();
    m_ptrFG_eval->setPlanner(m_strPlanner);
    m_ptrFG_eval->setCost(m_strCost);
}
CShort_Distance_Planner::~CShort_Distance_Planner()
{
    delete m_ptrFG_eval;
    m_ptrFG_eval = nullptr;
}

// spiral path plan

std::vector<double> CShort_Distance_Planner::cumulative_trapezoid(const std::vector<double>& vDiscrete_function_value,
                                                                  const std::vector<double>& vX_set,
                                                                  const double dInitial)
{
    std::vector<double> vCumulative_res;
    double dSum = dInitial;
    vCumulative_res.push_back(dSum);
    for (muint i = 0; i < vX_set.size() - 1; i++)
    {
        dSum += (vDiscrete_function_value[i] + vDiscrete_function_value[i + 1]) / 2.0 * (vX_set[i + 1] - vX_set[i]);
        vCumulative_res.push_back(dSum);
    }
    return vCumulative_res;
}

void CShort_Distance_Planner::spiral_path_finder(sPosition iStartPosition, sPosition iGoalPosition,
                                                 std::vector<double>& vSpiral_path_x,
                                                 std::vector<double>& vSpiral_path_y,
                                                 std::vector<double>& vSpiral_path_yaw,
                                                 std::vector<double>& vSpiral_path_curv, double& dMin_curvr,
                                                 bool& bSFlag, double& dS)
{
    size_t i;
    muint uSIndex;
    std::string strOptions;
    CppAD::ipopt::solve_result<Dvector> solution;  // solution

    // set solver options
    setSolverOptions(strOptions);

    if ("origin" == m_strPlanner)
    {
        size_t nx = 5;  // number of varibles
        size_t ng = 8;  // number of constraints
        Dvector v(nx);  // initial condition of varibles
        v[0] = 0.0;     // k0
        v[1] = 0.0;     // k1
        v[2] = 0.0;     // k2
        v[3] = 0.0;     // k3
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
        gl[1] = -m_dMax_curv;  // k1
        gu[1] = m_dMax_curv;
        gl[2] = -m_dMax_curv;  // k2
        gu[2] = m_dMax_curv;
        gl[3] = m_dMax_curv;  // k3
        gu[3] = m_dMax_curv;
        gl[4] = -10;  // s
        gu[4] = 10;
        gl[5] = 0;  // position_x-goal_x
        gu[5] = 0;
        gl[6] = 0;  // position_y-goal_y
        gu[6] = 0;
        gl[7] = 0;  // position_yaw-goal_yaw
        gu[7] = 0;

        // solve the problem
        CppAD::ipopt::solve<Dvector, CFG_eval>(strOptions, v, xl, xu, gl, gu, *m_ptrFG_eval, solution);
        uSIndex = 4;
    }
    else if ("soft" == m_strPlanner)
    {
        size_t nx = 5;  // number of varibles
        size_t ng = 5;  // number of constraints
        Dvector v(nx);  // initial condition of varibles
        v[0] = 0.0;     // k0
        v[1] = 0.0;     // k1
        v[2] = 0.0;     // k2
        v[3] = 0.0;     // k3
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
        gl[1] = -m_dMax_curv;  // k1
        gu[1] = m_dMax_curv;
        gl[2] = -m_dMax_curv;  // k2
        gu[2] = m_dMax_curv;
        gl[3] = -m_dMax_curv;  // k3
        gu[3] = m_dMax_curv;
        gl[4] = -20.0;  // s
        gu[4] = 20.0;

        // solve the problem
        CppAD::ipopt::solve<Dvector, CFG_eval>(strOptions, v, xl, xu, gl, gu, *m_ptrFG_eval, solution);
        uSIndex = 4;
    }
    else if ("dual_path" == m_strPlanner)
    {
        size_t nx = 13;  // number of varibles
        size_t ng = 22;  // number of constraints
        Dvector v(nx);   // initial condition of varibles
        v[0] = 0.0;      // k_front0
        v[1] = 0.0;      // k_front1
        v[2] = 0.0;      // k_front2
        v[3] = 0.0;      // k_front3
        v[4] = 10.0;     // s_front

        v[5] = 0.0;   // k_back0
        v[6] = 0.0;   // k_back1
        v[7] = 0.0;   // k_back2
        v[8] = 0.0;   // k_back3
        v[9] = 10.0;  // s_back

        v[10] = 0.0;  // middle_x
        v[11] = 0.0;  // middle_y
        v[12] = 0.0;  // middle_yaw

        // lower and upper bounds for varibles
        Dvector xl(nx), xu(nx);
        for (i = 0; i < nx; i++)
        {
            xl[i] = -100;
            xu[i] = +100;
        }

        Dvector gl(ng), gu(ng);
        gl[0] = 0;  // k_front0
        gu[0] = 0;
        gl[1] = -m_dMax_curv;  // k_front1
        gu[1] = m_dMax_curv;
        gl[2] = -m_dMax_curv;  // k_front2
        gu[2] = m_dMax_curv;
        gl[3] = -m_dMax_curv;  // k_front3
        gu[3] = m_dMax_curv;
        gl[4] = -10.0;  // s_front
        gu[4] = 10.0;
        gl[5] = -0.0;  // x_end_front-x_middle
        gu[5] = 0.0;
        gl[6] = -0.0;  // y_end_front-y_middle
        gu[6] = 0.0;
        gl[7] = -0.0;  // yaw_end_front-yaw_middle
        gu[7] = 0.0;

        gl[8] = 0;  // k_back0
        gu[8] = 0;
        gl[9] = -m_dMax_curv;  // k_back1
        gu[9] = m_dMax_curv;
        gl[10] = -m_dMax_curv;  // k_back2
        gu[10] = m_dMax_curv;
        gl[11] = -m_dMax_curv;  // k_back3
        gu[11] = m_dMax_curv;
        gl[12] = -10.0;  // s_back
        gu[12] = 10.0;
        gl[13] = -0.0;  // x_end_back-x_middle
        gu[13] = 0.0;
        gl[14] = -0.0;  // y_end_back-y_middle
        gu[14] = 0.0;
        gl[15] = -0.0;  // yaw_end_back-yaw_middle
        gu[15] = 0.0;

        gl[16] = 24.0;  // x_middle
        gu[16] = 30.0;
        gl[17] = -20.0;  // y_middle
        gu[17] = 20.0;
        gl[18] = std::max(iStartPosition.m_dYaw, iGoalPosition.m_dYaw) - 3.14159;  // yaw_middle
        gu[18] = std::min(iStartPosition.m_dYaw, iGoalPosition.m_dYaw) + 3.14159;
        gl[19] = -0.0;  // k_front3-k_back3
        gu[19] = 0.0;

        gl[20] = -0.35;  // InvalidS_front:-1.096
        gu[20] = 0.35;
        gl[21] = -0.35;  // InvalidS_back:-0.868
        gu[21] = 0.35;

        // solve the problem
        CppAD::ipopt::solve<Dvector, CFG_eval>(strOptions, v, xl, xu, gl, gu, *m_ptrFG_eval, solution);
        uSIndex = 4;
    }
    else if ("triple_path" == m_strPlanner)
    {
        size_t nx = 21;  // number of varibles
        size_t ng = 32;  // number of constraints
        Dvector v(nx);   // initial condition of varibles
        v[0] = 0.0;      // k_front0
        v[1] = 0.0;      // k_front1
        v[2] = 0.0;      // k_front2
        v[3] = 0.0;      // k_front3
        v[4] = 10.0;     // s_front

        v[5] = 0.0;   // k_inter0
        v[6] = 0.0;   // k_inter1
        v[7] = 0.0;   // k_inter2
        v[8] = 0.0;   // k_inter3
        v[9] = 10.0;  // s_inter

        v[10] = 0.0;   // k_back0
        v[11] = 0.0;   // k_back1
        v[12] = 0.0;   // k_back2
        v[13] = 0.0;   // k_back3
        v[14] = 10.0;  // s_back

        v[15] = 0.0;  // middle_x_front
        v[16] = 0.0;  // middle_y_front
        v[17] = 0.0;  // middle_yaw_front

        v[18] = 0.0;  // middle_x_back
        v[19] = 0.0;  // middle_y_back
        v[20] = 0.0;  // middle_yaw_back

        // lower and upper bounds for varibles
        Dvector xl(nx), xu(nx);
        for (i = 0; i < nx; i++)
        {
            xl[i] = -100;
            xu[i] = +100;
        }

        Dvector gl(ng), gu(ng);
        gl[0] = 0;  // k_front0
        gu[0] = 0;
        gl[1] = -m_dMax_curv;  // k_front1
        gu[1] = m_dMax_curv;
        gl[2] = -m_dMax_curv;  // k_front2
        gu[2] = m_dMax_curv;
        gl[3] = -m_dMax_curv;  // k_front3
        gu[3] = m_dMax_curv;
        gl[4] = -10.0;  // s_front
        gu[4] = 10.0;

        gl[5] = -0.0;  // x_end_front-x_middle_front
        gu[5] = 0.0;
        gl[6] = -0.0;  // y_end_front-y_middle_front
        gu[6] = 0.0;
        gl[7] = -0.0;  // yaw_end_front-yaw_middle_front
        gu[7] = 0.0;

        gl[8] = -m_dMax_curv;  // k_inter0
        gu[8] = m_dMax_curv;
        gl[9] = -m_dMax_curv;  // k_inter1
        gu[9] = m_dMax_curv;
        gl[10] = -m_dMax_curv;  // k_inter2
        gu[10] = m_dMax_curv;
        gl[11] = -m_dMax_curv;  // k_inter3
        gu[11] = m_dMax_curv;
        gl[12] = -10.0;  // s_inter
        gu[12] = 10.0;

        gl[13] = -0.0;  // x_middle_front-x_middle_back
        gu[13] = 0.0;
        gl[14] = -0.0;  // y_moddle_front-y_middle_back
        gu[14] = 0.0;
        gl[15] = -0.0;  // yaw_middle_front-yaw_middle_back
        gu[15] = 0.0;

        gl[16] = 24.0;  // x_middle_front
        gu[16] = 30.0;
        gl[17] = -20.0;  // y_middle_front
        gu[17] = 20.0;
        gl[18] = std::max(iStartPosition.m_dYaw, iGoalPosition.m_dYaw) - 3.14159;  // yaw_middle_front
        gu[18] = std::min(iStartPosition.m_dYaw, iGoalPosition.m_dYaw) + 3.14159;

        gl[19] = 24.0;  // x_middle_back
        gu[19] = 30.0;
        gl[20] = -20.0;  // y_middle_back
        gu[20] = 20.0;
        gl[21] = std::max(iStartPosition.m_dYaw, iGoalPosition.m_dYaw) - 3.14159;  // yaw_middle_back
        gu[21] = std::min(iStartPosition.m_dYaw, iGoalPosition.m_dYaw) + 3.14159;

        gl[22] = -0.0;  // k_back0
        gu[22] = 0.0;
        gl[23] = -m_dMax_curv;  // k_back1
        gu[23] = m_dMax_curv;
        gl[24] = -m_dMax_curv;  // k_back2
        gu[24] = m_dMax_curv;
        gl[25] = -m_dMax_curv;  // k_back3
        gu[25] = m_dMax_curv;
        gl[26] = -10.0;  // s_back
        gu[26] = 10.0;

        gl[27] = -0.0;  // x_end_back-x_middle_end
        gu[27] = 0.0;
        gl[28] = -0.0;  // y_end_back-y_middle_end
        gu[28] = 0.0;
        gl[29] = -0.0;  // yaw_end_back-yaw_middle_end
        gu[29] = 0.0;

        gl[30] = -0.0;  // k3_front-k0_inter
        gu[30] = 0.0;
        gl[31] = -0.0;  // k3_inter-k3_back
        gu[31] = 0.0;

        // solve the problem
        CppAD::ipopt::solve<Dvector, CFG_eval>(strOptions, v, xl, xu, gl, gu, *m_ptrFG_eval, solution);
        uSIndex = 4;
    }
    else if ("dual_path_with_limit_check" == m_strPlanner)
    {
        while ((m_uCurrent_iter <= m_uMax_iter) && false == m_bSolStatus)
        {
            MWARN("Current iter:%llu", m_uCurrent_iter);
            muint uInvalidSSize = m_ptrFG_eval->getInvalidSSize();
            size_t nx = 13;                  // number of varibles
            size_t ng = 20 + uInvalidSSize;  // number of constraints
            Dvector v(nx);                   // initial condition of varibles
            v[0] = 0.0;                      // k_front0
            v[1] = 0.0;                      // k_front1
            v[2] = 0.0;                      // k_front2
            v[3] = 0.0;                      // k_front3
            v[4] = 10.0;                     // s_front

            v[5] = 0.0;   // k_back0
            v[6] = 0.0;   // k_back1
            v[7] = 0.0;   // k_back2
            v[8] = 0.0;   // k_back3
            v[9] = 10.0;  // s_back

            v[10] = 0.0;  // middle_x
            v[11] = 0.0;  // middle_y
            v[12] = 0.0;  // middle_yaw

            // lower and upper bounds for varibles
            Dvector xl(nx), xu(nx);
            for (i = 0; i < nx; i++)
            {
                xl[i] = -100;
                xu[i] = +100;
            }

            Dvector gl(ng), gu(ng);
            gl[0] = 0;  // k_front0
            gu[0] = 0;
            gl[1] = -m_dMax_curv;  // k_front1
            gu[1] = m_dMax_curv;
            gl[2] = -m_dMax_curv;  // k_front2
            gu[2] = m_dMax_curv;
            gl[3] = -m_dMax_curv;  // k_front3
            gu[3] = m_dMax_curv;
            gl[4] = -10.0;  // s_front
            gu[4] = 10.0;
            gl[5] = -0.0;  // x_end_front-x_middle
            gu[5] = 0.0;
            gl[6] = -0.0;  // y_end_front-y_middle
            gu[6] = 0.0;
            gl[7] = -0.0;  // yaw_end_front-yaw_middle
            gu[7] = 0.0;

            gl[8] = 0;  // k_back0
            gu[8] = 0;
            gl[9] = -m_dMax_curv;  // k_back1
            gu[9] = m_dMax_curv;
            gl[10] = -m_dMax_curv;  // k_back2
            gu[10] = m_dMax_curv;
            gl[11] = -m_dMax_curv;  // k_back3
            gu[11] = m_dMax_curv;
            gl[12] = -10.0;  // s_back
            gu[12] = 10.0;
            gl[13] = -0.0;  // x_end_back-x_middle
            gu[13] = 0.0;
            gl[14] = -0.0;  // y_end_back-y_middle
            gu[14] = 0.0;
            gl[15] = -0.0;  // yaw_end_back-yaw_middle
            gu[15] = 0.0;

            gl[16] = 24.0;  // x_middle
            gu[16] = 30.0;
            gl[17] = -20.0;  // y_middle
            gu[17] = 20.0;
            gl[18] = std::max(iStartPosition.m_dYaw, iGoalPosition.m_dYaw) - 3.14159;  // yaw_middle
            gu[18] = std::min(iStartPosition.m_dYaw, iGoalPosition.m_dYaw) + 3.14159;
            gl[19] = -0.0;  // k_front3-k_back3
            gu[19] = 0.0;

            for (muint i = 0; i < uInvalidSSize; ++i)
            {
                gl[20 + i] = -0.35;  // k_invalidi
                gu[20 + i] = 0.35;
            }

            // solve the problem
            CppAD::ipopt::solve<Dvector, CFG_eval>(strOptions, v, xl, xu, gl, gu, *m_ptrFG_eval, solution);
            uSIndex = 4;
            ++m_uCurrent_iter;

            std::vector<AD<double>> res_a_front = m_ptrFG_eval->mapping_k2a(
                (solution.x)[0], (solution.x)[1], (solution.x)[2], (solution.x)[3], (solution.x)[4]);
            std::vector<AD<double>> res_a_back = m_ptrFG_eval->mapping_k2a(
                (solution.x)[5], (solution.x)[6], (solution.x)[7], (solution.x)[8], (solution.x)[9]);

            std::vector<double> vInvalidS_front;
            m_ptrFG_eval->checkCurv(res_a_front, m_dMax_curv, vInvalidS_front);
            std::vector<double> vInvalidS_back;
            m_ptrFG_eval->checkCurv(res_a_back, m_dMax_curv, vInvalidS_back);
            m_bSolStatus = ((vInvalidS_front.size() + vInvalidS_back.size()) == 0) ? true : false;
            if (m_bSolStatus)
            {
                break;
            }
            else
            {
                printInvalidS(vInvalidS_front, vInvalidS_back);
                m_ptrFG_eval->setInvalidS(vInvalidS_front, vInvalidS_back);
            }
        }
    }

    CalcPath(vSpiral_path_x, vSpiral_path_y, vSpiral_path_yaw, vSpiral_path_curv, solution, uSIndex, iStartPosition,
             iGoalPosition, dS, m_dMin_curv_r);
}

//********* callback function **************//

bool CShort_Distance_Planner::spiral_plan_srv_callback(spiral_planner::spiral::Request& request,
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
    m_ptrFG_eval->reset();
    m_ptrFG_eval->setStartPosition(iStartPosition);
    m_ptrFG_eval->setGoalPosition(iGoalPosition);
    std::vector<double> vSpiral_path_x, vSpiral_path_y, vSpiral_path_yaw, vSpiral_path_curv;
    double dMin_curvr, dS;
    bool bSFlag;

    ros::Time Current = ros::Time::now();
    spiral_path_finder(iStartPosition, iGoalPosition, vSpiral_path_x, vSpiral_path_y, vSpiral_path_yaw,
                       vSpiral_path_curv, dMin_curvr, bSFlag, dS);
    MINF("Spiral path planning time: %6.3f ms", 1000 * (ros::Time::now() - Current).toSec());
    MINF("Min_curvr:%6.3f", dMin_curvr);
    MINF("Total distance:%6.3f", dS);

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

void CShort_Distance_Planner::printSolStatus(const CppAD::ipopt::solve_result<Dvector>::status_type& enumStatusType)
{
    switch (enumStatusType)
    {
        case CppAD::ipopt::solve_result<Dvector>::status_type::not_defined:
        {
            MWARN("Solution status: not defined");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::success:
        {
            MWARN("Solution status: success");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::maxiter_exceeded:
        {
            MWARN("Solution status: maxiter_exceeded");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::stop_at_tiny_step:
        {
            MWARN("Solution status: stop_at_tiny_step");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::stop_at_acceptable_point:
        {
            MWARN("Solution status: stop_at_acceptable_point");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::local_infeasibility:
        {
            MWARN("Solution status: local_infeasibility");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::user_requested_stop:
        {
            MWARN("Solution status: user_requested_stop");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::feasible_point_found:
        {
            MWARN("Solution status: feasible_point_found");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::diverging_iterates:
        {
            MWARN("Solution status: diverging_iterates");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::restoration_failure:
        {
            MWARN("Solution status: restoration_failure");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::error_in_step_computation:
        {
            MWARN("Solution status: error_in_step_computation");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::invalid_number_detected:
        {
            MWARN("Solution status: invalid_number_detected");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::too_few_degrees_of_freedom:
        {
            MWARN("Solution status: too_few_degrees_of_freedom");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::internal_error:
        {
            MWARN("Solution status: internal_error");
            break;
        }
        case CppAD::ipopt::solve_result<Dvector>::status_type::unknown:
        {
            MWARN("Solution status: unknown");
            break;
        }
        default:
        {
            MERR("Unknown status type.");
            break;
        }
    }
}

void CShort_Distance_Planner::setSolverOptions(std::string& strOptions)
{
    // turn off any printing
    strOptions += "Integer print_level  0\n";
    strOptions += "String sb            yes\n";
    // maximum iterations
    strOptions += "Integer max_iter     1000\n";
    // approximate accuracy in first order necessary conditions;
    //  see Mathematical Programming, Volume 106, Number 1,
    //  Pages 25-57, Equation (6)
    strOptions += "Numeric tol          1e-6\n";
    // derivative tesing
    strOptions += "String derivative_test   second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    strOptions += "Numeric point_perturbation_radius   0.\n";
}

void CShort_Distance_Planner::CalcDiscretePath(const sPosition& iStartPosition, const double dS,
                                               const std::vector<AD<double>>& res_a,
                                               std::vector<double>& vSpiral_path_x, std::vector<double>& vSpiral_path_y,
                                               std::vector<double>& vSpiral_path_yaw,
                                               std::vector<double>& vSpiral_path_curv, double& dMin_curvr)
{
    vSpiral_path_x.clear();
    vSpiral_path_y.clear();
    vSpiral_path_yaw.clear();
    vSpiral_path_curv.clear();
    double dStep = 0.01, dMaxCurv = 0.0;
    int nSign = dS / fabs(dS);
    std::vector<double> vDiscrete_x, vDiscrete_y, vDiscrete_s;
    for (double i = 0; i < fabs(dS) + dStep; i += dStep)
    {
        double dDiscreteS = i * nSign;
        double theta_yaw = CppAD::Value(
            m_ptrFG_eval->theta(iStartPosition.m_dYaw, res_a[0], res_a[1], res_a[2], res_a[3], dDiscreteS));
        vDiscrete_x.push_back(cos(theta_yaw));
        vDiscrete_y.push_back(sin(theta_yaw));
        vSpiral_path_yaw.push_back(theta_yaw);
        vDiscrete_s.push_back(dDiscreteS);
        double dCurrenturv = CppAD::Value(m_ptrFG_eval->curv(res_a[0], res_a[1], res_a[2], res_a[3], dDiscreteS));
        vSpiral_path_curv.push_back(dCurrenturv);
        dMaxCurv = dMaxCurv > fabs(dCurrenturv) ? dMaxCurv : fabs(dCurrenturv);
    }

    vSpiral_path_x = cumulative_trapezoid(vDiscrete_x, vDiscrete_s, iStartPosition.m_dX);
    vSpiral_path_y = cumulative_trapezoid(vDiscrete_y, vDiscrete_s, iStartPosition.m_dY);
    dMin_curvr = 1.0 / (std::max(0.01, *max_element(vSpiral_path_curv.begin(), vSpiral_path_curv.end())));
}

void CShort_Distance_Planner::CalcSolFlag(const sPosition& iStartPosition, const sPosition& iGoalPosition,
                                          const double& dS, const double& dSpiral_path_last_x,
                                          const double& dSpiral_path_last_y, const double& dSpiral_path_last_yaw,
                                          bool& bSFlag)
{
    double dEuclideanDist =
        sqrt(pow(iStartPosition.m_dX - iGoalPosition.m_dX, 2) + pow(iStartPosition.m_dY - iGoalPosition.m_dY, 2));
    bool bSCost = dS < m_dS_cost_coef * dEuclideanDist;

    double dGoalError =
        sqrt(pow(dSpiral_path_last_x - iGoalPosition.m_dX, 2) + pow(dSpiral_path_last_y - iGoalPosition.m_dY, 2) +
             pow(dSpiral_path_last_yaw - iGoalPosition.m_dYaw, 2));
    bool bGoalCost = (dGoalError < m_Goal_cost_tolerence);

    if (false == bSCost)
    {
        MERR("S_cost false s:%6.3f,Euclidean dist cost:%6.3f", dS, m_dS_cost_coef * dEuclideanDist);
    }
    if (false == bGoalCost)
    {
        MERR("X_Error,Y_Error,Yaw_Error: (%6.3f, %6.3f, %6.3f)", fabs(dSpiral_path_last_x - iGoalPosition.m_dX),
             fabs(dSpiral_path_last_y - iGoalPosition.m_dY), fabs(dSpiral_path_last_yaw - iGoalPosition.m_dYaw));
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

void CShort_Distance_Planner::PathCombination(std::vector<double>& vSpiral_path_x, std::vector<double>& vSpiral_path_y,
                                              std::vector<double>& vSpiral_path_yaw,
                                              std::vector<double>& vSpiral_path_curv,
                                              const std::vector<double>& vSpiral_path_x_front,
                                              const std::vector<double>& vSpiral_path_y_front,
                                              const std::vector<double>& vSpiral_path_yaw_front,
                                              const std::vector<double>& vSpiral_path_curv_front, const bool bIsReverse)
{
    if (bIsReverse)
    {
        vSpiral_path_x.insert(vSpiral_path_x.end(), vSpiral_path_x_front.rbegin(), vSpiral_path_x_front.rend());
        vSpiral_path_y.insert(vSpiral_path_y.end(), vSpiral_path_y_front.rbegin(), vSpiral_path_y_front.rend());
        vSpiral_path_yaw.insert(vSpiral_path_yaw.end(), vSpiral_path_yaw_front.rbegin(), vSpiral_path_yaw_front.rend());
        vSpiral_path_curv.insert(vSpiral_path_curv.end(), vSpiral_path_curv_front.rbegin(),
                                 vSpiral_path_curv_front.rend());
    }
    else
    {
        vSpiral_path_x.insert(vSpiral_path_x.end(), vSpiral_path_x_front.begin(), vSpiral_path_x_front.end());
        vSpiral_path_y.insert(vSpiral_path_y.end(), vSpiral_path_y_front.begin(), vSpiral_path_y_front.end());
        vSpiral_path_yaw.insert(vSpiral_path_yaw.end(), vSpiral_path_yaw_front.begin(), vSpiral_path_yaw_front.end());
        vSpiral_path_curv.insert(vSpiral_path_curv.end(), vSpiral_path_curv_front.begin(),
                                 vSpiral_path_curv_front.end());
    }
}

void CShort_Distance_Planner::printInvalidS(const std::vector<double>& vInvalidS_front,
                                            const std::vector<double>& vInvalidS_back)
{
    for (muint i = 0; i < vInvalidS_front.size(); ++i)
    {
        MWARN("InvalidS front:%6.3f", vInvalidS_front[i]);
    }
    for (muint i = 0; i < vInvalidS_back.size(); ++i)
    {
        MWARN("InvalidS back:%6.3f", vInvalidS_back[i]);
    }
}

void CShort_Distance_Planner::CalcPath(std::vector<double>& vSpiral_path_x, std::vector<double>& vSpiral_path_y,
                                       std::vector<double>& vSpiral_path_yaw, std::vector<double>& vSpiral_path_curv,
                                       CppAD::ipopt::solve_result<Dvector> solution, const muint uSIndex,
                                       const sPosition& iStartPosition, const sPosition& iGoalPosition, double& dS,
                                       double& dMin_curvr)
{
    if ("dual_path" == m_strPlanner)
    {
        std::vector<AD<double>> res_a_front = m_ptrFG_eval->mapping_k2a(
            (solution.x)[0], (solution.x)[1], (solution.x)[2], (solution.x)[3], (solution.x)[4]);
        std::vector<AD<double>> res_a_back = m_ptrFG_eval->mapping_k2a(
            (solution.x)[5], (solution.x)[6], (solution.x)[7], (solution.x)[8], (solution.x)[9]);
        MINF("obj_value:%6.3f", solution.obj_value);
        printSolStatus(solution.status);

        double dS_front = CppAD::Value(res_a_front[uSIndex]);
        double dMin_curvr_front;
        std::vector<double> vSpiral_path_x_front, vSpiral_path_y_front, vSpiral_path_yaw_front, vSpiral_path_curv_front;
        CalcDiscretePath(iStartPosition, dS_front, res_a_front, vSpiral_path_x_front, vSpiral_path_y_front,
                         vSpiral_path_yaw_front, vSpiral_path_curv_front, dMin_curvr_front);

        double dS_back = CppAD::Value(res_a_back[uSIndex]);
        double dMin_curvr_back;
        std::vector<double> vSpiral_path_x_back, vSpiral_path_y_back, vSpiral_path_yaw_back, vSpiral_path_curv_back;
        CalcDiscretePath(iGoalPosition, dS_back, res_a_back, vSpiral_path_x_back, vSpiral_path_y_back,
                         vSpiral_path_yaw_back, vSpiral_path_curv_back, dMin_curvr_back);
        dS = fabs(dS_front) + fabs(dS_back);
        dMin_curvr = std::min(dMin_curvr_front, dMin_curvr_back);

        PathCombination(vSpiral_path_x, vSpiral_path_y, vSpiral_path_yaw, vSpiral_path_curv, vSpiral_path_x_front,
                        vSpiral_path_y_front, vSpiral_path_yaw_front, vSpiral_path_curv_front, false);
        PathCombination(vSpiral_path_x, vSpiral_path_y, vSpiral_path_yaw, vSpiral_path_curv, vSpiral_path_x_back,
                        vSpiral_path_y_back, vSpiral_path_yaw_back, vSpiral_path_curv_back, true);
    }
    else if ("triple_path" == m_strPlanner)
    {
        std::vector<AD<double>> res_a_front = m_ptrFG_eval->mapping_k2a(
            (solution.x)[0], (solution.x)[1], (solution.x)[2], (solution.x)[3], (solution.x)[4]);
        std::vector<AD<double>> res_a_inter = m_ptrFG_eval->mapping_k2a(
            (solution.x)[5], (solution.x)[6], (solution.x)[7], (solution.x)[8], (solution.x)[9]);
        std::vector<AD<double>> res_a_back = m_ptrFG_eval->mapping_k2a(
            (solution.x)[10], (solution.x)[11], (solution.x)[12], (solution.x)[13], (solution.x)[14]);
        double dMiddle_x_front = (solution.x)[15];
        double dMiddle_y_front = (solution.x)[16];
        double dMiddle_yaw_front = (solution.x)[17];
        sPosition iMiddlePose_front(dMiddle_x_front, dMiddle_y_front, dMiddle_yaw_front);
        MINF("obj_value:%6.3f", solution.obj_value);
        printSolStatus(solution.status);

        double dS_front = CppAD::Value(res_a_front[uSIndex]);
        double dMin_curvr_front;
        std::vector<double> vSpiral_path_x_front, vSpiral_path_y_front, vSpiral_path_yaw_front, vSpiral_path_curv_front;
        CalcDiscretePath(iStartPosition, dS_front, res_a_front, vSpiral_path_x_front, vSpiral_path_y_front,
                         vSpiral_path_yaw_front, vSpiral_path_curv_front, dMin_curvr_front);

        double dS_inter = CppAD::Value(res_a_inter[uSIndex]);
        double dMin_curvr_inter;
        std::vector<double> vSpiral_path_x_inter, vSpiral_path_y_inter, vSpiral_path_yaw_inter, vSpiral_path_curv_inter;
        CalcDiscretePath(iMiddlePose_front, dS_inter, res_a_inter, vSpiral_path_x_inter, vSpiral_path_y_inter,
                         vSpiral_path_yaw_inter, vSpiral_path_curv_inter, dMin_curvr_inter);

        double dS_back = CppAD::Value(res_a_back[uSIndex]);
        double dMin_curvr_back;
        std::vector<double> vSpiral_path_x_back, vSpiral_path_y_back, vSpiral_path_yaw_back, vSpiral_path_curv_back;
        CalcDiscretePath(iGoalPosition, dS_back, res_a_back, vSpiral_path_x_back, vSpiral_path_y_back,
                         vSpiral_path_yaw_back, vSpiral_path_curv_back, dMin_curvr_back);
        dS = fabs(dS_front) + fabs(dS_inter) + fabs(dS_back);
        dMin_curvr = std::min(std::min(dMin_curvr_front, dMin_curvr_inter), dMin_curvr_back);

        PathCombination(vSpiral_path_x, vSpiral_path_y, vSpiral_path_yaw, vSpiral_path_curv, vSpiral_path_x_front,
                        vSpiral_path_y_front, vSpiral_path_yaw_front, vSpiral_path_curv_front, false);
        PathCombination(vSpiral_path_x, vSpiral_path_y, vSpiral_path_yaw, vSpiral_path_curv, vSpiral_path_x_inter,
                        vSpiral_path_y_inter, vSpiral_path_yaw_inter, vSpiral_path_curv_inter, false);
        PathCombination(vSpiral_path_x, vSpiral_path_y, vSpiral_path_yaw, vSpiral_path_curv, vSpiral_path_x_back,
                        vSpiral_path_y_back, vSpiral_path_yaw_back, vSpiral_path_curv_back, true);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spiral_planner");
    CShort_Distance_Planner iSDP;

    ros::spin();

    return 0;
}
