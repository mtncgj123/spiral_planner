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

double CFG_eval::cost(const double& a0, const double& a1, const double& a2, const double& a3, const double& s)
{
    double dS_cost_coef = 0.01;
    return (pow(a3, 2) / 7.0 * pow(s, 7) + 2 / 6.0 * a3 * a2 * pow(s, 6) +
            (pow(a2, 2) + 2 * a3 * a1 + 9 * pow(a3, 2)) / 5.0 * pow(s, 5) +
            (2 * a3 * a0 + 2 * a2 * a1 + 12 * a2 * a3) / 4.0 * pow(s, 4) +
            (pow(a1, 2) + 2 * a2 * a0 + 4 * pow(a2, 2) + 6 * a1 * a3) / 3.0 * pow(s, 3) +
            (2 * a1 * a0 + 4 * a1 * a2) / 2.0 * pow(s, 2) + (pow(a0, 2) + pow(a1, 2)) * s) *
               s / fabs(s) +
           dS_cost_coef * s * s;
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

    if ("origin" == m_strPlanner)
    {
        return (pow(a3, 2) / 7.0 * pow(s, 7) + 2 / 6.0 * a3 * a2 * pow(s, 6) +
                (pow(a2, 2) + 2 * a3 * a1 + 9 * pow(a3, 2)) / 5.0 * pow(s, 5) +
                (2 * a3 * a0 + 2 * a2 * a1 + 12 * a2 * a3) / 4.0 * pow(s, 4) +
                (pow(a1, 2) + 2 * a2 * a0 + 4 * pow(a2, 2) + 6 * a1 * a3) / 3.0 * pow(s, 3) +
                (2 * a1 * a0 + 4 * a1 * a2) / 2.0 * pow(s, 2) + (pow(a0, 2) + pow(a1, 2)) * s) *
               s / fabs(s);
    }
    else if ("soft" == m_strPlanner)
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
    else if ("dual_path" == m_strPlanner)
    {
        return (pow(a3, 2) / 7.0 * pow(s, 7) + 2 / 6.0 * a3 * a2 * pow(s, 6) +
                (pow(a2, 2) + 2 * a3 * a1 + 9 * pow(a3, 2)) / 5.0 * pow(s, 5) +
                (2 * a3 * a0 + 2 * a2 * a1 + 12 * a2 * a3) / 4.0 * pow(s, 4) +
                (pow(a1, 2) + 2 * a2 * a0 + 4 * pow(a2, 2) + 6 * a1 * a3) / 3.0 * pow(s, 3) +
                (2 * a1 * a0 + 4 * a1 * a2) / 2.0 * pow(s, 2) + (pow(a0, 2) + pow(a1, 2)) * s) *
                   s / fabs(s) +
               dS_cost_coef * s * s;
    }
    else if ("dual_path_with_limit_check" == m_strPlanner)
    {
        return (pow(a3, 2) / 7.0 * pow(s, 7) + 2 / 6.0 * a3 * a2 * pow(s, 6) +
                (pow(a2, 2) + 2 * a3 * a1 + 9 * pow(a3, 2)) / 5.0 * pow(s, 5) +
                (2 * a3 * a0 + 2 * a2 * a1 + 12 * a2 * a3) / 4.0 * pow(s, 4) +
                (pow(a1, 2) + 2 * a2 * a0 + 4 * pow(a2, 2) + 6 * a1 * a3) / 3.0 * pow(s, 3) +
                (2 * a1 * a0 + 4 * a1 * a2) / 2.0 * pow(s, 2) + (pow(a0, 2) + pow(a1, 2)) * s) *
                   s / fabs(s) +
               dS_cost_coef * s * s;
    }
    else if ("triple_path" == m_strPlanner)
    {
        return (pow(a3, 2) / 7.0 * pow(s, 7) + 2 / 6.0 * a3 * a2 * pow(s, 6) +
                (pow(a2, 2) + 2 * a3 * a1 + 9 * pow(a3, 2)) / 5.0 * pow(s, 5) +
                (2 * a3 * a0 + 2 * a2 * a1 + 12 * a2 * a3) / 4.0 * pow(s, 4) +
                (pow(a1, 2) + 2 * a2 * a0 + 4 * pow(a2, 2) + 6 * a1 * a3) / 3.0 * pow(s, 3) +
                (2 * a1 * a0 + 4 * a1 * a2) / 2.0 * pow(s, 2) + (pow(a0, 2) + pow(a1, 2)) * s) *
                   s / fabs(s) +
               dS_cost_coef * s * s;
    }
    else if ("multi_bezier_path" == m_strPlanner)
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
        MERR("Planner type invalid");
        return s;
    }
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

void CFG_eval::checkCurv(const std::vector<AD<double>>& res_a, const double& dMaxCurv, std::vector<double>& vecInvalidS)
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
            vecInvalidS.push_back(dSol_s);
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
                vecInvalidS.push_back(dSol_s1);
            }
        }
        if (dCurv2 < -dMaxCurv || dCurv2 > dMaxCurv)
        {
            if (0 <= dSol_s2 / dS && dSol_s2 / dS <= 1)
            {
                vecInvalidS.push_back(dSol_s2);
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
        assert(fg.size() == 23);
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
        fg[21] = curv(polyparam_set_front[0], polyparam_set_front[1], polyparam_set_front[2], polyparam_set_front[3],
                      -1.096);
        fg[22] =
            curv(polyparam_set_back[0], polyparam_set_back[1], polyparam_set_back[2], polyparam_set_back[3], -0.868);

        return;
    }
    else if ("dual_path_with_limit_check" == m_strPlanner)
    {
        muint uSize_vecInvalidS_front = m_vecInvalidS_front.size();
        muint uSize_vecInvalidS_back = m_vecInvalidS_back.size();
        MERR("CFG_eval invalidS size:%lld", uSize_vecInvalidS_front + uSize_vecInvalidS_back);
        assert(p.size() == 13);
        assert(fg.size() == 21 + uSize_vecInvalidS_front + uSize_vecInvalidS_back);
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

        for (muint i = 0; i < uSize_vecInvalidS_front; ++i)
        {
            fg[21 + i] = curv(polyparam_set_front[0], polyparam_set_front[1], polyparam_set_front[2],
                              polyparam_set_front[3], m_vecInvalidS_front[i]);
            MWARN("vecInvalidS_fronti:%6.3f", m_vecInvalidS_front[i]);
        }
        for (muint i = 0; i < uSize_vecInvalidS_back; ++i)
        {
            fg[21 + uSize_vecInvalidS_front + i] =
                curv(polyparam_set_back[0], polyparam_set_back[1], polyparam_set_back[2], polyparam_set_back[3],
                     m_vecInvalidS_back[i]);
            MWARN("vecInvalidS_backi:%6.3f", m_vecInvalidS_back[i]);
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
    else if ("multi_bezier_path" == m_strPlanner)
    {
        assert(p.size() == 53);
        assert(fg.size() == 82);
        std::vector<AD<double>> vec_p;
        vec_p.resize(53);
        for (muint i = 0; i < vec_p.size(); ++i)
        {
            vec_p[i] = p[i];
        }

        fg[0] = pow(cost(vec_p[0], vec_p[1], vec_p[2], vec_p[3], vec_p[4]), 2) +
                pow(cost(vec_p[5], vec_p[6], vec_p[7], vec_p[8], vec_p[9]), 2) +
                pow(cost(vec_p[10], vec_p[11], vec_p[12], vec_p[13], vec_p[14]), 2) +
                pow(cost(vec_p[15], vec_p[16], vec_p[17], vec_p[18], vec_p[19]), 2) +
                pow(cost(vec_p[20], vec_p[21], vec_p[22], vec_p[23], vec_p[24]), 2) +
                pow(cost(vec_p[25], vec_p[26], vec_p[27], vec_p[28], vec_p[29]), 2) +
                pow(cost(vec_p[30], vec_p[31], vec_p[32], vec_p[33], vec_p[34]), 2);

        for (muint i = 0; i < 53; ++i)
        {
            fg[i] = p[i];
        }
        fg[53] = position_x(m_iStartPosition.m_dX, m_iStartPosition.m_dYaw, p[0], p[1], p[2], p[3], p[4]) -
                 p[35];  // middle_0
        fg[54] = position_y(m_iStartPosition.m_dY, m_iStartPosition.m_dYaw, p[0], p[1], p[2], p[3], p[4]) - p[36];
        fg[55] = theta(m_iStartPosition.m_dYaw, p[0], p[1], p[2], p[3], p[4]) - p[37];

        fg[56] = position_x(p[35], p[37], p[5], p[6], p[7], p[8], p[9]) - p[38];  // middle_1
        fg[57] = position_y(p[36], p[37], p[5], p[6], p[7], p[8], p[9]) - p[39];
        fg[58] = theta(p[37], p[5], p[6], p[7], p[8], p[9]) - p[40];

        fg[59] = position_x(p[38], p[40], p[10], p[11], p[12], p[13], p[14]) - p[41];  // middle_2
        fg[60] = position_y(p[39], p[40], p[10], p[11], p[12], p[13], p[14]) - p[42];
        fg[61] = theta(p[40], p[10], p[11], p[12], p[13], p[14]) - p[43];

        fg[62] = position_x(p[41], p[43], p[15], p[16], p[17], p[18], p[19]) - p[44];  // middle_3
        fg[63] = position_y(p[42], p[43], p[15], p[16], p[17], p[18], p[19]) - p[45];
        fg[64] = theta(p[43], p[15], p[16], p[17], p[18], p[19]) - p[46];

        fg[65] = position_x(p[44], p[46], p[20], p[21], p[22], p[23], p[24]) - p[47];  // middle_4
        fg[66] = position_y(p[45], p[46], p[20], p[21], p[22], p[23], p[24]) - p[48];
        fg[67] = theta(p[46], p[20], p[21], p[22], p[23], p[24]) - p[49];

        fg[68] = position_x(p[47], p[49], p[25], p[26], p[27], p[28], p[29]) - p[50];  // middle_5
        fg[69] = position_y(p[48], p[49], p[25], p[26], p[27], p[28], p[29]) - p[51];
        fg[70] = theta(p[49], p[25], p[26], p[27], p[28], p[29]) - p[52];

        fg[71] = position_x(m_iGoalPosition.m_dX, m_iGoalPosition.m_dYaw, p[30], p[31], p[32], p[33], p[34]) -
                 p[50];  // middle_5_back
        fg[72] = position_y(m_iGoalPosition.m_dY, m_iGoalPosition.m_dYaw, p[30], p[31], p[32], p[33], p[34]) - p[51];
        fg[73] = theta(m_iGoalPosition.m_dYaw, p[30], p[31], p[32], p[33], p[34]) - p[52];

        fg[74] = curv(p[0], p[1], p[2], p[3], p[4]) - p[5];                                          // middle_curv_0
        fg[75] = curv(p[5], p[6], p[7], p[8], p[9]) - p[10];                                         // middle_curv_1
        fg[76] = curv(p[10], p[11], p[12], p[13], p[14]) - p[15];                                    // middle_curv_2
        fg[77] = curv(p[15], p[16], p[17], p[18], p[19]) - p[20];                                    // middle_curv_3
        fg[78] = curv(p[20], p[21], p[22], p[23], p[24]) - p[25];                                    // middle_curv_4
        fg[79] = curv(p[25], p[26], p[27], p[28], p[29]) - curv(p[30], p[31], p[32], p[33], p[34]);  // middle_curv_5

        fg[80] = p[0];
        fg[81] = p[30];
    }
}

void CFG_eval::reset()
{
    setStartPosition(sPosition(0, 0, 0));
    setGoalPosition(sPosition(0, 0, 0));
    m_vecInvalidS_front.clear();
    m_vecInvalidS_back.clear();
}

// CShort_Distance_Planner function definition
CShort_Distance_Planner::CShort_Distance_Planner()
{
    m_servSpiralPlanner =
        m_nh.advertiseService("spiral_plan_request", &CShort_Distance_Planner::spiral_plan_srv_callback, this);

    m_nh.param("/spiral_planner/S_cost_coef", m_dS_cost_coef, 100.0);
    m_nh.param("/spiral_planner/Goal_cost_tolerence", m_Goal_cost_tolerence, 100.0);
    m_nh.param("/spiral_planner/planner", m_strPlanner, std::string("invalid"));

    MINF("S_cost_coef %6.3f", m_dS_cost_coef);
    MINF("Goal_cost_tolerence %6.3f", m_Goal_cost_tolerence);
    MINF("m_strPlanner %s", m_strPlanner.c_str());

    m_ptrFG_eval = new CFG_eval();
    m_ptrFG_eval->setPlanner(m_strPlanner);
}
CShort_Distance_Planner::~CShort_Distance_Planner()
{
    delete m_ptrFG_eval;
    m_ptrFG_eval = nullptr;
}

// spiral path plan

std::vector<double> CShort_Distance_Planner::cumulative_trapezoid(const std::vector<double>& vecDiscrete_function_value,
                                                                  const std::vector<double>& vecX_set,
                                                                  const double dInitial)
{
    std::vector<double> vecCumulative_res;
    double dSum = dInitial;
    vecCumulative_res.push_back(dSum);
    for (muint i = 0; i < vecX_set.size() - 1; i++)
    {
        dSum +=
            (vecDiscrete_function_value[i] + vecDiscrete_function_value[i + 1]) / 2.0 * (vecX_set[i + 1] - vecX_set[i]);
        vecCumulative_res.push_back(dSum);
    }
    return vecCumulative_res;
}

void CShort_Distance_Planner::spiral_path_finder(sPosition iStartPosition, sPosition iGoalPosition,
                                                 std::vector<double>& vecSpiral_path_x,
                                                 std::vector<double>& vecSpiral_path_y,
                                                 std::vector<double>& vecSpiral_path_yaw,
                                                 std::vector<double>& vecSpiral_path_curv, double& dMin_curvr,
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
        gl[4] = -20;  // s
        gu[4] = 20;
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
    else if ("multi_bezier_path" == m_strPlanner)
    {
        size_t nx = 53;  // number of varibles
        size_t ng = 81;  // number of constraints
        Dvector v(nx);   // initial condition of varibles

        for (muint i = 0; i < 53; ++i)
        {
            v[i] = 1.0;
        }

        // lower and upper bounds for varibles
        Dvector xl(nx), xu(nx);
        for (i = 0; i < nx; i++)
        {
            xl[i] = -5;
            xu[i] = +5;
        }

        m_dMax_curv = 10.0;

        Dvector gl(ng), gu(ng);
        gl[0] = -m_dMax_curv;  // k_front0
        gu[0] = m_dMax_curv;
        gl[1] = -m_dMax_curv;  // k_front1
        gu[1] = m_dMax_curv;
        gl[2] = -m_dMax_curv;  // k_front2
        gu[2] = m_dMax_curv;
        gl[3] = -m_dMax_curv;  // k_front3
        gu[3] = m_dMax_curv;
        gl[4] = -1.0;  // s_front
        gu[4] = 1.0;

        gl[5] = -m_dMax_curv;  // k_front0
        gu[5] = m_dMax_curv;
        gl[6] = -m_dMax_curv;  // k_front1
        gu[6] = m_dMax_curv;
        gl[7] = -m_dMax_curv;  // k_front2
        gu[7] = m_dMax_curv;
        gl[8] = -m_dMax_curv;  // k_front3
        gu[8] = m_dMax_curv;
        gl[9] = -1.0;  // s_front
        gu[9] = 1.0;

        gl[10] = -m_dMax_curv;  // k_front0
        gu[10] = m_dMax_curv;
        gl[11] = -m_dMax_curv;  // k_front1
        gu[11] = m_dMax_curv;
        gl[12] = -m_dMax_curv;  // k_front2
        gu[12] = m_dMax_curv;
        gl[13] = -m_dMax_curv;  // k_front3
        gu[13] = m_dMax_curv;
        gl[14] = -1.0;  // s_front
        gu[14] = 1.0;

        gl[15] = -m_dMax_curv;  // k_front0
        gu[15] = m_dMax_curv;
        gl[16] = -m_dMax_curv;  // k_front1
        gu[16] = m_dMax_curv;
        gl[17] = -m_dMax_curv;  // k_front2
        gu[17] = m_dMax_curv;
        gl[18] = -m_dMax_curv;  // k_front3
        gu[18] = m_dMax_curv;
        gl[19] = -1.0;  // s_front
        gu[19] = 1.0;

        gl[20] = -m_dMax_curv;  // k_front0
        gu[20] = m_dMax_curv;
        gl[21] = -m_dMax_curv;  // k_front1
        gu[21] = m_dMax_curv;
        gl[22] = -m_dMax_curv;  // k_front2
        gu[22] = m_dMax_curv;
        gl[23] = -m_dMax_curv;  // k_front3
        gu[23] = m_dMax_curv;
        gl[24] = -1.0;  // s_front
        gu[24] = 1.0;

        gl[25] = -m_dMax_curv;  // k_front0
        gu[25] = m_dMax_curv;
        gl[26] = -m_dMax_curv;  // k_front1
        gu[26] = m_dMax_curv;
        gl[27] = -m_dMax_curv;  // k_front2
        gu[27] = m_dMax_curv;
        gl[28] = -m_dMax_curv;  // k_front3
        gu[28] = m_dMax_curv;
        gl[29] = -1.0;  // s_front
        gu[29] = 1.0;

        gl[30] = -m_dMax_curv;  // k_front0
        gu[30] = m_dMax_curv;
        gl[31] = -m_dMax_curv;  // k_front1
        gu[31] = m_dMax_curv;
        gl[32] = -m_dMax_curv;  // k_front2
        gu[32] = m_dMax_curv;
        gl[33] = -m_dMax_curv;  // k_front3
        gu[33] = m_dMax_curv;
        gl[34] = -1.0;  // s_front
        gu[34] = 1.0;

        muint uIndex_1 = 35;
        while (uIndex_1 <= 52)
        {
            gl[uIndex_1] = 24.0;  // x_middle_back
            gu[uIndex_1] = 30.0;
            ++uIndex_1;
            gl[uIndex_1] = -20.0;  // y_middle_back
            gu[uIndex_1] = 20.0;
            ++uIndex_1;
            gl[uIndex_1] = std::max(iStartPosition.m_dYaw, iGoalPosition.m_dYaw) - 3.14159;  // yaw_middle_back
            gu[uIndex_1] = std::min(iStartPosition.m_dYaw, iGoalPosition.m_dYaw) + 3.14159;
            ++uIndex_1;
        }

        for (muint i = 52; i <= 80; ++i)
        {
            gl[i] = -0.0;  // x_end_back-x_middle_end
            gu[i] = 0.0;
        }

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

            std::vector<double> vecInvalidS_front;
            m_ptrFG_eval->checkCurv(res_a_front, m_dMax_curv, vecInvalidS_front);
            std::vector<double> vecInvalidS_back;
            m_ptrFG_eval->checkCurv(res_a_back, m_dMax_curv, vecInvalidS_back);
            m_bSolStatus = ((vecInvalidS_front.size() + vecInvalidS_back.size()) == 0) ? true : false;
            if (m_bSolStatus)
            {
                break;
            }
            else
            {
                printInvalidS(vecInvalidS_front, vecInvalidS_back);
                m_ptrFG_eval->setInvalidS(vecInvalidS_front, vecInvalidS_back);
            }
        }
    }

    CalcPath(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv, solution, uSIndex,
             iStartPosition, iGoalPosition, dS, m_dMin_curv_r);
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
    std::vector<double> vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv;
    double dMin_curvr, dS;
    bool bSFlag;

    ros::Time Current = ros::Time::now();
    spiral_path_finder(iStartPosition, iGoalPosition, vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw,
                       vecSpiral_path_curv, dMin_curvr, bSFlag, dS);
    MINF("Spiral path planning time: %6.3f ms", 1000 * (ros::Time::now() - Current).toSec());
    MINF("Min_curvr:%6.3f", dMin_curvr);
    MINF("Total distance:%6.3f", dS);

    geometry_msgs::PoseArray traj;
    for (muint i = 0; i < vecSpiral_path_x.size(); i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = vecSpiral_path_x[i];
        pose.position.y = vecSpiral_path_y[i];
        pose.orientation.z = vecSpiral_path_yaw[i];
        pose.orientation.w = vecSpiral_path_curv[i];
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
                                               std::vector<double>& vecSpiral_path_x,
                                               std::vector<double>& vecSpiral_path_y,
                                               std::vector<double>& vecSpiral_path_yaw,
                                               std::vector<double>& vecSpiral_path_curv, double& dMin_curvr)
{
    vecSpiral_path_x.clear();
    vecSpiral_path_y.clear();
    vecSpiral_path_yaw.clear();
    vecSpiral_path_curv.clear();
    double dStep = 0.01, dMaxCurv = 0.0;
    int nSign = dS / fabs(dS);
    std::vector<double> vecDiscrete_x, vecDiscrete_y, vecDiscrete_s;
    for (double i = 0; i < fabs(dS) + dStep; i += dStep)
    {
        double dDiscreteS = i * nSign;
        double theta_yaw = CppAD::Value(
            m_ptrFG_eval->theta(iStartPosition.m_dYaw, res_a[0], res_a[1], res_a[2], res_a[3], dDiscreteS));
        vecDiscrete_x.push_back(cos(theta_yaw));
        vecDiscrete_y.push_back(sin(theta_yaw));
        vecSpiral_path_yaw.push_back(theta_yaw);
        vecDiscrete_s.push_back(dDiscreteS);
        double dCurrenturv = CppAD::Value(m_ptrFG_eval->curv(res_a[0], res_a[1], res_a[2], res_a[3], dDiscreteS));
        vecSpiral_path_curv.push_back(dCurrenturv);
        dMaxCurv = dMaxCurv > fabs(dCurrenturv) ? dMaxCurv : fabs(dCurrenturv);
    }

    vecSpiral_path_x = cumulative_trapezoid(vecDiscrete_x, vecDiscrete_s, iStartPosition.m_dX);
    vecSpiral_path_y = cumulative_trapezoid(vecDiscrete_y, vecDiscrete_s, iStartPosition.m_dY);
    dMin_curvr = 1.0 / (std::max(0.01, *max_element(vecSpiral_path_curv.begin(), vecSpiral_path_curv.end())));
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

void CShort_Distance_Planner::PathCombination(
    std::vector<double>& vecSpiral_path_x, std::vector<double>& vecSpiral_path_y,
    std::vector<double>& vecSpiral_path_yaw, std::vector<double>& vecSpiral_path_curv,
    const std::vector<double>& vecSpiral_path_x_front, const std::vector<double>& vecSpiral_path_y_front,
    const std::vector<double>& vecSpiral_path_yaw_front, const std::vector<double>& vecSpiral_path_curv_front,
    const bool bIsReverse)
{
    if (bIsReverse)
    {
        vecSpiral_path_x.insert(vecSpiral_path_x.end(), vecSpiral_path_x_front.rbegin(), vecSpiral_path_x_front.rend());
        vecSpiral_path_y.insert(vecSpiral_path_y.end(), vecSpiral_path_y_front.rbegin(), vecSpiral_path_y_front.rend());
        vecSpiral_path_yaw.insert(vecSpiral_path_yaw.end(), vecSpiral_path_yaw_front.rbegin(),
                                  vecSpiral_path_yaw_front.rend());
        vecSpiral_path_curv.insert(vecSpiral_path_curv.end(), vecSpiral_path_curv_front.rbegin(),
                                   vecSpiral_path_curv_front.rend());
    }
    else
    {
        vecSpiral_path_x.insert(vecSpiral_path_x.end(), vecSpiral_path_x_front.begin(), vecSpiral_path_x_front.end());
        vecSpiral_path_y.insert(vecSpiral_path_y.end(), vecSpiral_path_y_front.begin(), vecSpiral_path_y_front.end());
        vecSpiral_path_yaw.insert(vecSpiral_path_yaw.end(), vecSpiral_path_yaw_front.begin(),
                                  vecSpiral_path_yaw_front.end());
        vecSpiral_path_curv.insert(vecSpiral_path_curv.end(), vecSpiral_path_curv_front.begin(),
                                   vecSpiral_path_curv_front.end());
    }
}

void CShort_Distance_Planner::printInvalidS(const std::vector<double>& vecInvalidS_front,
                                            const std::vector<double>& vecInvalidS_back)
{
    for (muint i = 0; i < vecInvalidS_front.size(); ++i)
    {
        MWARN("InvalidS front:%6.3f", vecInvalidS_front[i]);
    }
    for (muint i = 0; i < vecInvalidS_back.size(); ++i)
    {
        MWARN("InvalidS back:%6.3f", vecInvalidS_back[i]);
    }
}

void CShort_Distance_Planner::CalcPath(std::vector<double>& vecSpiral_path_x, std::vector<double>& vecSpiral_path_y,
                                       std::vector<double>& vecSpiral_path_yaw,
                                       std::vector<double>& vecSpiral_path_curv,
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
        std::vector<double> vecSpiral_path_x_front, vecSpiral_path_y_front, vecSpiral_path_yaw_front,
            vecSpiral_path_curv_front;
        CalcDiscretePath(iStartPosition, dS_front, res_a_front, vecSpiral_path_x_front, vecSpiral_path_y_front,
                         vecSpiral_path_yaw_front, vecSpiral_path_curv_front, dMin_curvr_front);

        double dS_back = CppAD::Value(res_a_back[uSIndex]);
        double dMin_curvr_back;
        std::vector<double> vecSpiral_path_x_back, vecSpiral_path_y_back, vecSpiral_path_yaw_back,
            vecSpiral_path_curv_back;
        CalcDiscretePath(iGoalPosition, dS_back, res_a_back, vecSpiral_path_x_back, vecSpiral_path_y_back,
                         vecSpiral_path_yaw_back, vecSpiral_path_curv_back, dMin_curvr_back);
        dS = fabs(dS_front) + fabs(dS_back);
        dMin_curvr = std::min(dMin_curvr_front, dMin_curvr_back);

        PathCombination(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv,
                        vecSpiral_path_x_front, vecSpiral_path_y_front, vecSpiral_path_yaw_front,
                        vecSpiral_path_curv_front, false);
        PathCombination(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv,
                        vecSpiral_path_x_back, vecSpiral_path_y_back, vecSpiral_path_yaw_back, vecSpiral_path_curv_back,
                        true);
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
        std::vector<double> vecSpiral_path_x_front, vecSpiral_path_y_front, vecSpiral_path_yaw_front,
            vecSpiral_path_curv_front;
        CalcDiscretePath(iStartPosition, dS_front, res_a_front, vecSpiral_path_x_front, vecSpiral_path_y_front,
                         vecSpiral_path_yaw_front, vecSpiral_path_curv_front, dMin_curvr_front);

        double dS_inter = CppAD::Value(res_a_inter[uSIndex]);
        double dMin_curvr_inter;
        std::vector<double> vecSpiral_path_x_inter, vecSpiral_path_y_inter, vecSpiral_path_yaw_inter,
            vecSpiral_path_curv_inter;
        CalcDiscretePath(iMiddlePose_front, dS_inter, res_a_inter, vecSpiral_path_x_inter, vecSpiral_path_y_inter,
                         vecSpiral_path_yaw_inter, vecSpiral_path_curv_inter, dMin_curvr_inter);

        double dS_back = CppAD::Value(res_a_back[uSIndex]);
        double dMin_curvr_back;
        std::vector<double> vecSpiral_path_x_back, vecSpiral_path_y_back, vecSpiral_path_yaw_back,
            vecSpiral_path_curv_back;
        CalcDiscretePath(iGoalPosition, dS_back, res_a_back, vecSpiral_path_x_back, vecSpiral_path_y_back,
                         vecSpiral_path_yaw_back, vecSpiral_path_curv_back, dMin_curvr_back);
        dS = fabs(dS_front) + fabs(dS_inter) + fabs(dS_back);
        dMin_curvr = std::min(std::min(dMin_curvr_front, dMin_curvr_inter), dMin_curvr_back);

        PathCombination(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv,
                        vecSpiral_path_x_front, vecSpiral_path_y_front, vecSpiral_path_yaw_front,
                        vecSpiral_path_curv_front, false);
        PathCombination(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv,
                        vecSpiral_path_x_inter, vecSpiral_path_y_inter, vecSpiral_path_yaw_inter,
                        vecSpiral_path_curv_inter, false);
        PathCombination(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv,
                        vecSpiral_path_x_back, vecSpiral_path_y_back, vecSpiral_path_yaw_back, vecSpiral_path_curv_back,
                        true);
    }
    else if ("multi_bezier_path" == m_strPlanner)
    {
        printSolStatus(solution.status);
        for (muint i = 0; i < 35; ++i)
        {
            MWARN("solution [%llu]: %6.3f", i, (solution.x)[i]);
        }
        MINF("obj_value:%6.3f", solution.obj_value);
        double dS0 = (solution.x)[4];
        double dMin_curvr_0;
        double dCost =
            m_ptrFG_eval->cost((solution.x)[0], (solution.x)[1], (solution.x)[2], (solution.x)[3], (solution.x)[4]) +
            m_ptrFG_eval->cost((solution.x)[5], (solution.x)[6], (solution.x)[7], (solution.x)[8], (solution.x)[9]) +
            m_ptrFG_eval->cost((solution.x)[10], (solution.x)[11], (solution.x)[12], (solution.x)[13],
                               (solution.x)[14]) +
            m_ptrFG_eval->cost((solution.x)[15], (solution.x)[16], (solution.x)[17], (solution.x)[18],
                               (solution.x)[19]) +
            m_ptrFG_eval->cost((solution.x)[20], (solution.x)[21], (solution.x)[22], (solution.x)[23],
                               (solution.x)[24]) +
            m_ptrFG_eval->cost((solution.x)[25], (solution.x)[26], (solution.x)[27], (solution.x)[28],
                               (solution.x)[29]) +
            m_ptrFG_eval->cost((solution.x)[30], (solution.x)[31], (solution.x)[32], (solution.x)[33],
                               (solution.x)[34]);
        MWARN("TOTAL COST:%6.3f", dCost);
        std::vector<AD<double>> res_a_0 = {(solution.x)[0], (solution.x)[1], (solution.x)[2], (solution.x)[3],
                                           (solution.x)[4]};
        std::vector<double> vecSpiral_path_x_0, vecSpiral_path_y_0, vecSpiral_path_yaw_0, vecSpiral_path_curv_0;
        CalcDiscretePath(iStartPosition, dS0, res_a_0, vecSpiral_path_x_0, vecSpiral_path_y_0, vecSpiral_path_yaw_0,
                         vecSpiral_path_curv_0, dMin_curvr_0);

        double dS1 = (solution.x)[9];
        double dMin_curvr_1;
        double dMiddle_x_0 = (solution.x)[35];
        double dMiddle_y_0 = (solution.x)[36];
        double dMiddle_yaw_0 = (solution.x)[37];
        sPosition iMiddlePose_0(dMiddle_x_0, dMiddle_y_0, dMiddle_yaw_0);
        std::vector<AD<double>> res_a_1 = {(solution.x)[5], (solution.x)[6], (solution.x)[7], (solution.x)[8],
                                           (solution.x)[9]};
        std::vector<double> vecSpiral_path_x_1, vecSpiral_path_y_1, vecSpiral_path_yaw_1, vecSpiral_path_curv_1;
        CalcDiscretePath(iMiddlePose_0, dS1, res_a_1, vecSpiral_path_x_1, vecSpiral_path_y_1, vecSpiral_path_yaw_1,
                         vecSpiral_path_curv_1, dMin_curvr_1);

        double dS2 = (solution.x)[14];
        double dMin_curvr_2;
        double dMiddle_x_1 = (solution.x)[38];
        double dMiddle_y_1 = (solution.x)[39];
        double dMiddle_yaw_1 = (solution.x)[40];
        sPosition iMiddlePose_1(dMiddle_x_1, dMiddle_y_1, dMiddle_yaw_1);
        std::vector<AD<double>> res_a_2 = {(solution.x)[10], (solution.x)[11], (solution.x)[12], (solution.x)[13],
                                           (solution.x)[14]};
        std::vector<double> vecSpiral_path_x_2, vecSpiral_path_y_2, vecSpiral_path_yaw_2, vecSpiral_path_curv_2;
        CalcDiscretePath(iMiddlePose_1, dS2, res_a_2, vecSpiral_path_x_2, vecSpiral_path_y_2, vecSpiral_path_yaw_2,
                         vecSpiral_path_curv_2, dMin_curvr_2);

        double dS3 = (solution.x)[19];
        double dMin_curvr_3;
        double dMiddle_x_2 = (solution.x)[41];
        double dMiddle_y_2 = (solution.x)[42];
        double dMiddle_yaw_2 = (solution.x)[43];
        sPosition iMiddlePose_2(dMiddle_x_2, dMiddle_y_2, dMiddle_yaw_2);
        std::vector<AD<double>> res_a_3 = {(solution.x)[15], (solution.x)[16], (solution.x)[17], (solution.x)[18],
                                           (solution.x)[19]};
        std::vector<double> vecSpiral_path_x_3, vecSpiral_path_y_3, vecSpiral_path_yaw_3, vecSpiral_path_curv_3;
        CalcDiscretePath(iMiddlePose_2, dS3, res_a_3, vecSpiral_path_x_3, vecSpiral_path_y_3, vecSpiral_path_yaw_3,
                         vecSpiral_path_curv_3, dMin_curvr_3);

        double dS4 = (solution.x)[24];
        double dMin_curvr_4;
        double dMiddle_x_3 = (solution.x)[44];
        double dMiddle_y_3 = (solution.x)[45];
        double dMiddle_yaw_3 = (solution.x)[46];
        sPosition iMiddlePose_3(dMiddle_x_3, dMiddle_y_3, dMiddle_yaw_3);
        std::vector<AD<double>> res_a_4 = {(solution.x)[20], (solution.x)[21], (solution.x)[22], (solution.x)[23],
                                           (solution.x)[24]};
        std::vector<double> vecSpiral_path_x_4, vecSpiral_path_y_4, vecSpiral_path_yaw_4, vecSpiral_path_curv_4;
        CalcDiscretePath(iMiddlePose_3, dS4, res_a_4, vecSpiral_path_x_4, vecSpiral_path_y_4, vecSpiral_path_yaw_4,
                         vecSpiral_path_curv_4, dMin_curvr_4);

        double dS5 = (solution.x)[29];
        double dMin_curvr_5;
        double dMiddle_x_4 = (solution.x)[47];
        double dMiddle_y_4 = (solution.x)[48];
        double dMiddle_yaw_4 = (solution.x)[49];
        sPosition iMiddlePose_4(dMiddle_x_4, dMiddle_y_4, dMiddle_yaw_4);
        std::vector<AD<double>> res_a_5 = {(solution.x)[25], (solution.x)[26], (solution.x)[27], (solution.x)[28],
                                           (solution.x)[29]};
        std::vector<double> vecSpiral_path_x_5, vecSpiral_path_y_5, vecSpiral_path_yaw_5, vecSpiral_path_curv_5;
        CalcDiscretePath(iMiddlePose_4, dS5, res_a_5, vecSpiral_path_x_5, vecSpiral_path_y_5, vecSpiral_path_yaw_5,
                         vecSpiral_path_curv_5, dMin_curvr_5);

        double dS6 = (solution.x)[34];
        double dMin_curvr_6;
        std::vector<AD<double>> res_a_6 = {(solution.x)[30], (solution.x)[31], (solution.x)[32], (solution.x)[33],
                                           (solution.x)[34]};
        std::vector<double> vecSpiral_path_x_6, vecSpiral_path_y_6, vecSpiral_path_yaw_6, vecSpiral_path_curv_6;
        CalcDiscretePath(iGoalPosition, dS5, res_a_6, vecSpiral_path_x_6, vecSpiral_path_y_6, vecSpiral_path_yaw_6,
                         vecSpiral_path_curv_6, dMin_curvr_6);

        dS = fabs(dS0) + fabs(dS1) + fabs(dS2) + fabs(dS3) + fabs(dS4) + fabs(dS5) + fabs(dS6);

        // dMin_curvr = std::min(std::min(dMin_curvr_front, dMin_curvr_inter), dMin_curvr_back);

        PathCombination(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv, vecSpiral_path_x_0,
                        vecSpiral_path_y_0, vecSpiral_path_yaw_0, vecSpiral_path_curv_0, false);
        PathCombination(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv, vecSpiral_path_x_1,
                        vecSpiral_path_y_1, vecSpiral_path_yaw_1, vecSpiral_path_curv_1, false);
        PathCombination(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv, vecSpiral_path_x_2,
                        vecSpiral_path_y_2, vecSpiral_path_yaw_2, vecSpiral_path_curv_2, false);
        PathCombination(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv, vecSpiral_path_x_3,
                        vecSpiral_path_y_3, vecSpiral_path_yaw_3, vecSpiral_path_curv_3, false);
        PathCombination(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv, vecSpiral_path_x_4,
                        vecSpiral_path_y_4, vecSpiral_path_yaw_4, vecSpiral_path_curv_4, false);
        PathCombination(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv, vecSpiral_path_x_5,
                        vecSpiral_path_y_5, vecSpiral_path_yaw_5, vecSpiral_path_curv_5, false);
        PathCombination(vecSpiral_path_x, vecSpiral_path_y, vecSpiral_path_yaw, vecSpiral_path_curv, vecSpiral_path_x_6,
                        vecSpiral_path_y_6, vecSpiral_path_yaw_6, vecSpiral_path_curv_6, true);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spiral_planner");
    CShort_Distance_Planner iSDP;

    ros::spin();

    return 0;
}
