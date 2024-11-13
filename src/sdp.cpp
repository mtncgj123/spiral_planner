#include <math.h>

#include <algorithm>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <vector>

#include "Spiral_Planner.h"

using namespace std;
using CppAD::AD;

/**
 * @brief :Construct a new CFGEval::CFGEval object
 *        Objective function and Gradient evaluation
 *
 * @param iStartPose :Start pose
 * @param iGoalPose :Goal pose
 */
CFGEval::CFGEval(CPose2D iStartPose, CPose2D iGoalPose)
{
    m_iStartPose = iStartPose;
    m_iGoalPose = iGoalPose;
}

/**
 * @brief :Calculate curvature
 *
 * @param ADdX_0 :Polynomial coefficient
 * @param ADdX_1 :Polynomial coefficient
 * @param ADdX_2 :Polynomial coefficient
 * @param ADdX_3 :Polynomial coefficient
 * @param ADdX_4 :Polynomial coefficient
 * @param sStartPose :Start pose
 * @param sGoalPose :Goal pose
 * @return AD<mdouble> :Curvature
 */
AD<mdouble> CFGEval::CalCurv(const AD<mdouble>& ADdX_0, const AD<mdouble>& ADdX_1, const AD<mdouble>& ADdX_2,
                             const AD<mdouble>& ADdX_3, const AD<mdouble>& ADdS)
{
    return ADdX_3 * pow(ADdS, 3) + ADdX_2 * pow(ADdS, 2) + ADdX_1 * ADdS + ADdX_0;
}

/**
 * @brief :Calculate theta
 *
 */
AD<mdouble> CFGEval::CalTheta(mdouble dTheta_0, const AD<mdouble>& ADdX_0, const AD<mdouble>& ADdX_1,
                              const AD<mdouble>& ADdX_2, const AD<mdouble>& ADdX_3, const AD<mdouble>& ADds)
{
    return dTheta_0 + ADdX_3 * pow(ADds, 4) / 4 + ADdX_2 * pow(ADds, 3) / 3 + ADdX_1 * pow(ADds, 2) / 2 + ADdX_0 * ADds;
}

/**
 * @brief Calculate x coordinate using simpson integration
 */
AD<mdouble> CFGEval::CalPoseX(mdouble dPoseX_0, mdouble dTheta_0, const AD<mdouble>& ADdX_0, const AD<mdouble>& ADdX_1,
                              const AD<mdouble>& ADdX_2, const AD<mdouble>& ADdX_3, const AD<mdouble>& ADds)
{
    return dPoseX_0 + ADds / 24 *
                          (cos(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, 0)) +
                           4 * cos(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds / 8)) +
                           2 * cos(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 2 / 8)) +
                           4 * cos(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 3 / 8)) +
                           2 * cos(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 4 / 8)) +
                           4 * cos(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 5 / 8)) +
                           2 * cos(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 6 / 8)) +
                           4 * cos(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 7 / 8)) +
                           cos(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 8 / 8)));
}

/**
 * @brief Calculate y coordinate using simpson integration
 */
AD<mdouble> CFGEval::CalPoseY(mdouble dPoseY_0, mdouble dTheta_0, const AD<mdouble>& ADdX_0, const AD<mdouble>& ADdX_1,
                              const AD<mdouble>& ADdX_2, const AD<mdouble>& ADdX_3, const AD<mdouble>& ADds)
{
    return dPoseY_0 + ADds / 24 *
                          (sin(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, 0)) +
                           4 * sin(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds / 8)) +
                           2 * sin(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 2 / 8)) +
                           4 * sin(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 3 / 8)) +
                           2 * sin(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 4 / 8)) +
                           4 * sin(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 5 / 8)) +
                           2 * sin(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 6 / 8)) +
                           4 * sin(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 7 / 8)) +
                           sin(CalTheta(dTheta_0, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADds * 8 / 8)));
}

/**
 * @brief Cost function
 *
 */

AD<mdouble> CFGEval::Cost(const AD<mdouble>& ADdX_0, const AD<mdouble>& ADdX_1, const AD<mdouble>& ADdX_2,
                          const AD<mdouble>& ADdX_3, const AD<mdouble>& ADdX_4, CPose2D sStartPose, CPose2D sGoalPose)
{
    // mdouble dXInit = sStartPose.m_dx;
    // mdouble dYInit = sStartPose.m_dy;
    // mdouble dYawInit = sStartPose.m_dRz;

    // mdouble dXGoal = sGoalPose.m_dx;
    // mdouble dYGoal = sGoalPose.m_dy;
    // mdouble dYawGoal = sGoalPose.m_dRz;

    // curvature cost and goal Pose cost
    // return (pow(x3, 2) / 7 * pow(x4, 7) + pow(x2, 2) / 5 * pow(x4, 5) + pow(x1, 2) / 3 * pow(x4, 3) + pow(x0, 2) * x4
    // +
    //         2 / 6 * x2 * x3 * pow(x4, 6) + 2 / 4 * x1 * x2 * pow(x4, 4) + 2 / 3 * x0 * x2 * pow(x4, 3) +
    //         2 / 5 * x3 * x1 * pow(x4, 5) + 2 / 4 * x3 * x0 * pow(x4, 4) + x1 * x0 * pow(x4, 2) +
    //         100 * pow(Pose_x(x_init, yaw_init, x0, x1, x2, x3, x4) - x_goal, 2) +
    //         100 * pow(Pose_y(y_init, yaw_init, x0, x1, x2, x3, x4) - y_goal, 2) +
    //         100 * pow(theta(yaw_init, x0, x1, x2, x3, x4) - yaw_goal, 2));

    // curvature cost, deltacurvature cost and goal Pose cost
    // 10:cost coeficcient,magic number
    return (pow(ADdX_3, 2) / 7.0 * pow(ADdX_4, 7) + 2.0 / 6.0 * ADdX_3 * ADdX_2 * pow(ADdX_4, 6) +
            (pow(ADdX_2, 2) + 2 * ADdX_3 * ADdX_1 + 9 * pow(ADdX_3, 2)) / 5.0 * pow(ADdX_4, 5) +
            (2 * ADdX_3 * ADdX_0 + 2 * ADdX_2 * ADdX_1 + 12 * ADdX_2 * ADdX_3) / 4.0 * pow(ADdX_4, 4) +
            (pow(ADdX_1, 2) + 2 * ADdX_2 * ADdX_0 + 4 * pow(ADdX_2, 2) + 6 * ADdX_1 * ADdX_3) / 3.0 * pow(ADdX_4, 3) +
            (2 * ADdX_1 * ADdX_0 + 4 * ADdX_1 * ADdX_2) / 2.0 * pow(ADdX_4, 2) +
            (pow(ADdX_0, 2) + pow(ADdX_1, 2)) * ADdX_4);

    // 100 * pow(CalPoseX(dXInit, dYawInit, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADdX_4) - dXGoal, 2) +
    // 100 * pow(CalPoseY(dYInit, dYawInit, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADdX_4) - dYGoal, 2) +
    // 100 * pow(CalTheta(dYawInit, ADdX_0, ADdX_1, ADdX_2, ADdX_3, ADdX_4) - dYawGoal, 2));
}

/**
 * @brief Mapping function convert P to X
 */
vector<AD<mdouble>> CFGEval::MappingP2X(const AD<mdouble>& ADdP_0, const AD<mdouble>& ADdP_1, const AD<mdouble>& ADdP_2,
                                        const AD<mdouble>& ADdP_3, const AD<mdouble>& ADdP_4)
{
    vector<AD<mdouble>> vMapX;
    AD<mdouble> ADdX_1 = -(11.0 / 2.0 * ADdP_0 - 9 * ADdP_1 + 9.0 / 2.0 * ADdP_2 - ADdP_3) / ADdP_4;
    AD<mdouble> ADdX_2 = (9 * ADdP_0 - 45.0 / 2.0 * ADdP_1 + 18 * ADdP_2 - 9.0 / 2.0 * ADdP_3) / pow(ADdP_4, 2);
    AD<mdouble> ADdX_3 =
        -(9.0 / 2.0 * ADdP_0 - 27.0 / 2.0 * ADdP_1 + 27.0 / 2.0 * ADdP_2 - 9.0 / 2.0 * ADdP_3) / pow(ADdP_4, 3);
    vMapX.push_back(ADdP_0);
    vMapX.push_back(ADdX_1);
    vMapX.push_back(ADdX_2);
    vMapX.push_back(ADdX_3);
    vMapX.push_back(ADdP_4);
    return vMapX;
}

void CFGEval::operator()(ADvector& fg, const ADvector& p)
{
    assert(fg.size() == 9);
    assert(p.size() == 5);
    // variables
    AD<mdouble> ADdP_0 = p[0];
    AD<mdouble> ADdP_1 = p[1];
    AD<mdouble> ADdP_2 = p[2];
    AD<mdouble> ADdP_3 = p[3];
    AD<mdouble> ADdP_4 = p[4];

    // f(x) objective function
    vector<AD<mdouble>> vXSet = MappingP2X(ADdP_0, ADdP_1, ADdP_2, ADdP_3, ADdP_4);

    fg[0] = Cost(vXSet[0], vXSet[1], vXSet[2], vXSet[3], vXSet[4], m_iStartPose, m_iGoalPose);

    // constraints
    fg[1] = ADdP_0;
    fg[2] = pow(m_dMaxCurv, 2) - pow(ADdP_1, 2);
    fg[3] = pow(m_dMaxCurv, 2) - pow(ADdP_2, 2);
    fg[4] = pow(m_dMaxCurv, 2) - pow(ADdP_3, 2);
    fg[5] = ADdP_4;
    fg[6] = CalPoseX(m_iStartPose.m_dx, m_iStartPose.m_dRz, vXSet[0], vXSet[1], vXSet[2], vXSet[3], vXSet[4]) -
            m_iGoalPose.m_dx;
    fg[7] = CalPoseY(m_iStartPose.m_dy, m_iStartPose.m_dRz, vXSet[0], vXSet[1], vXSet[2], vXSet[3], vXSet[4]) -
            m_iGoalPose.m_dy;
    fg[8] = CalTheta(m_iStartPose.m_dRz, vXSet[0], vXSet[1], vXSet[2], vXSet[3], vXSet[4]) - m_iGoalPose.m_dRz;
    return;
}

// CSpiralPlanner function definition

/**
 * @brief Calculate circle origin direction
 *
 * @param iStartPose :Start pose
 * @param iGoalPose :Goal pose
 * @return mint :Flag to show circle origin direction
 */
mint CSpiralPlanner::CircleOriginFlag(CPose2D iStartPose, CPose2D iGoalPose)
{
    vector<mdouble> vdStartDir = {cos(iStartPose.m_dRz), sin(iStartPose.m_dRz)};
    vector<mdouble> vdGoalDir = {cos(iGoalPose.m_dRz), sin(iGoalPose.m_dRz)};

    if (vdStartDir[0] * vdGoalDir[1] - vdStartDir[1] * vdGoalDir[0] >= 0)
    {
        return -1;
    }
    return 1;
}

/**
 * @brief :Calculate circle origin
 *
 * @param iStartPose :Start pose
 * @param dR :Radius
 * @param nOriFlag :Circle origin flag
 * @return CPose2D :Circle origin pose
 */
CPose2D CSpiralPlanner::CircleOriginFinder(CPose2D iStartPose, mdouble dR, mint nOriFlag)
{
    mdouble dStartPoseX = iStartPose.m_dx;
    mdouble dStartPoseY = iStartPose.m_dy;
    mdouble dStartPoseYaw = iStartPose.m_dRz;

    mdouble dThetaOrigin = dStartPoseYaw + M_PI / 2 * nOriFlag;
    mdouble dOriginX = dStartPoseX + dR * cos(dThetaOrigin);
    mdouble dOriginY = dStartPoseY + dR * sin(dThetaOrigin);
    CPose2D iOrigin;
    iOrigin.m_dx = dOriginX;
    iOrigin.m_dy = dOriginY;
    iOrigin.m_dRz = 0;

    return iOrigin;
}

/**
 * @brief Calculate circle path
 *
 * @param iStartPose :Start pose
 * @param iOriginPose :Circle origin pose
 * @param dCirclePathGoalYaw :Circle path goal yaw
 * @param nOriFlag :Circle path origin flag
 * @return vector<CPose2D> :Circle path
 */
vector<CPose2D> CSpiralPlanner::CirclePathPlanner(CPose2D iStartPose, CPose2D iOriginPose, mdouble dCirclePathGoalYaw,
                                                  mint nOriFlag)
{
    mdouble dThetaOri2Goal = dCirclePathGoalYaw - M_PI / 2 * nOriFlag;
    mint nRotationDirection;
    vector<CPose2D> viCirclePath;

    mdouble dStartPoseX = iStartPose.m_dx;
    mdouble dStartPoseY = iStartPose.m_dy;
    mdouble dStartPoseYaw = iStartPose.m_dRz;

    mdouble dOriginX = iOriginPose.m_dx;
    mdouble dOriginY = iOriginPose.m_dy;

    vector<mdouble> vdDirOri2Start = {dStartPoseX - dOriginX, dStartPoseY - dOriginY};
    vector<mdouble> vdDirOri2Goal = {cos(dThetaOri2Goal), sin(dThetaOri2Goal)};

    mdouble dTmpThetaRotation = (vdDirOri2Start[0] * vdDirOri2Goal[0] + vdDirOri2Start[1] * vdDirOri2Goal[1]) /
                                (sqrt(pow(vdDirOri2Start[0], 2) + pow(vdDirOri2Start[1], 2)) *
                                 sqrt(pow(vdDirOri2Goal[0], 2) + pow(vdDirOri2Goal[1], 2)));

    mdouble dThetaRotation = acos(min(max(dTmpThetaRotation, -1.0), 1.0));  // In case dTmpThetaRotation out of range

    if (vdDirOri2Start[0] * vdDirOri2Goal[1] - vdDirOri2Start[1] * vdDirOri2Goal[0] > 0)
    {
        nRotationDirection = 1;
    }
    else
    {
        nRotationDirection = -1;
    }

    mdouble dNThetaStep = 100.0;
    CPose2D iTmpPose;
    for (muint i = 0; i < dNThetaStep + 1; i++)
    {
        iTmpPose.m_dx = dOriginX +
                        cos(i / dNThetaStep * dThetaRotation * nRotationDirection) * (dStartPoseX - dOriginX) -
                        sin(i / dNThetaStep * dThetaRotation * nRotationDirection) * (dStartPoseY - dOriginY);

        iTmpPose.m_dy = dOriginY +
                        sin(i / dNThetaStep * dThetaRotation * nRotationDirection) * (dStartPoseX - dOriginX) +
                        cos(i / dNThetaStep * dThetaRotation * nRotationDirection) * (dStartPoseY - dOriginY);

        iTmpPose.m_dRz = dStartPoseYaw + i / dNThetaStep * dThetaRotation * nRotationDirection;

        viCirclePath.push_back(iTmpPose);
    }
    return viCirclePath;
}

/**
 * @brief Calculate line path trajectory
 *
 * @param iStartPose :Start pose
 * @param nDirection :Line path direction(forward or backward)
 * @param dDistance :Line path distance
 * @return vector<CPose2D> Line path
 */
vector<CPose2D> CSpiralPlanner::LinePathPlanner(CPose2D iStartPose, mint nDirection, mdouble dDistance)
{
    mdouble dStartPoseX = iStartPose.m_dx;
    mdouble dStartPoseY = iStartPose.m_dy;
    mdouble dStartPoseYaw = iStartPose.m_dRz;
    vector<CPose2D> viLinePath;
    mdouble dMovingYaw;

    if (-1 == nDirection)
    {
        dMovingYaw = dStartPoseYaw + M_PI;
    }
    else
    {
        dMovingYaw = dStartPoseYaw;
    }
    mdouble dPathStep = 0.25;
    mint nNPathStep = (mint)(dDistance / dPathStep);
    for (mint i = 0; i < nNPathStep + 1; i++)
    {
        CPose2D iTmpPose;
        iTmpPose.m_dx = dStartPoseX + i * dPathStep * cos(dMovingYaw);
        iTmpPose.m_dy = dStartPoseY + i * dPathStep * sin(dMovingYaw);
        iTmpPose.m_dRz = dStartPoseYaw;
        viLinePath.push_back(iTmpPose);
    }
    return viLinePath;
}

/**
 * @brief :Decide whether it is proper to generate spiral path given
 *
 * @param iStartPose :Start pose
 * @param iGoalPose :Goal pose
 * @return mbool
 */
mbool CSpiralPlanner::isPolyable(CPose2D iStartPose, CPose2D iGoalPose)
{
    mdouble dStartPoseX = iStartPose.m_dx;
    mdouble dStartPoseY = iStartPose.m_dy;
    mdouble dStartPoseYaw = iStartPose.m_dRz;
    vector<mdouble> vdStartPoseDirection = {cos(dStartPoseYaw), sin(dStartPoseYaw)};

    mdouble dGoalPoseX = iGoalPose.m_dx;
    mdouble dGoalPoseY = iGoalPose.m_dy;
    mdouble dGoalPoseYaw = iGoalPose.m_dRz;

    vector<mdouble> vdGoalPoseDirection = {cos(dGoalPoseYaw), sin(dGoalPoseYaw)};
    vector<mdouble> vdDirectionStart2Goal = {dGoalPoseX - dStartPoseX, dGoalPoseY - dStartPoseY};

    mdouble dYawTest =
        vdStartPoseDirection[0] * vdGoalPoseDirection[0] + vdStartPoseDirection[1] * vdGoalPoseDirection[1];

    mdouble dOptimalTest_1 =
        vdDirectionStart2Goal[0] * vdStartPoseDirection[1] - vdDirectionStart2Goal[1] * vdStartPoseDirection[0];
    mdouble dOptimalTest_2 =
        vdDirectionStart2Goal[0] * vdGoalPoseDirection[1] - vdDirectionStart2Goal[1] * vdGoalPoseDirection[0];

    mdouble dSubOptimalTest_1 =
        vdGoalPoseDirection[0] * vdStartPoseDirection[1] - vdGoalPoseDirection[1] * vdStartPoseDirection[0];
    mdouble dSubOptimalTest_2 =
        vdGoalPoseDirection[0] * vdDirectionStart2Goal[1] - vdGoalPoseDirection[1] * vdDirectionStart2Goal[0];

    if (dYawTest >= 0 && (dOptimalTest_1 * dOptimalTest_2 < 0 || dSubOptimalTest_1 * dSubOptimalTest_2 < 0))
    {
        return true;
    }
    return false;
}

/**
 * @brief Calculate integration vector
 *
 * @param vdXDiscrete :Set for discrete x
 * @param vdSSet :Set for discrete s
 * @param dInitial :Integration initial data
 * @return vector<mdouble> :Integration
 */
vector<mdouble> CSpiralPlanner::CumulativeTrapezoid(const vector<mdouble>& vdXDiscrete, const vector<mdouble>& vdSSet,
                                                    mdouble dInitial)
{
    vector<mdouble> vdCumulativeRes;
    mdouble dSum = dInitial;
    vdCumulativeRes.push_back(dSum);
    for (mint i = 0; i < (mint)vdSSet.size() - 1; i++)
    {
        dSum += (vdXDiscrete[i] + vdXDiscrete[i + 1]) / 2 * (vdSSet[i + 1] - vdSSet[i]);
        vdCumulativeRes.push_back(dSum);
    }
    return vdCumulativeRes;
}

/**
 * @brief Convert an angle to 0~2pi
 *
 * @param dTheta
 * @return mdouble
 */
mdouble CSpiralPlanner::ConvertTheta(mdouble dTheta)
{
    double dTmpTheta = CBasicMath::RadianTailor(dTheta);
    if (dTmpTheta < 0)
    {
        dTmpTheta += 2 * M_PI;
    }
    return dTmpTheta;
}

/**
 * @brief :Calculate spiral path
 *
 * @param iStartPose :Start pose
 * @param iGoalPose :Goal pose
 * @return tuple<vector<CPose2D>,mdouble,mbool> :SPiral path, minimum radius, planning result flag
 */
tuple<vector<CPose2D>, mdouble, mbool> CSpiralPlanner::SpiralPathFinder(CPose2D iStartPose, CPose2D iGoalPose)
{
    // Convert start point yaw and goal point yaw to [0,2pi)
    // Goal cost function need a uniform heading angle criteria
    iStartPose.m_dRz = ConvertTheta(iStartPose.m_dRz);
    iGoalPose.m_dRz = ConvertTheta(iGoalPose.m_dRz);

    typedef CPPAD_TESTVECTOR(mdouble) Dvector;
    m_iFGEval = CFGEval(iStartPose, iGoalPose);
    mdouble dIniDistanceGuess = sqrt(pow(iStartPose.m_dx - iGoalPose.m_dx, 2) +
                                     pow(iStartPose.m_dy - iGoalPose.m_dy, 2));  // Distance initial guess

    size_t i;

    size_t nx = 5;   // number of varibles
    size_t ng = 8;   // number of constraints
    Dvector p0(nx);  // initial condition of varibles
    p0[0] = 0.0;
    p0[1] = 0.0;
    p0[2] = 0.0;
    p0[3] = 0.0;
    p0[4] = dIniDistanceGuess;

    // lower and upper bounds for varibles
    Dvector xl(nx), xu(nx);
    for (i = 0; i < nx; i++)
    {
        xl[i] = -100;
        xu[i] = +100;
    }
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
    gu[4] = 1.0e19;
    gl[5] = 0;
    gu[5] = 0;
    gl[6] = 0;
    gu[6] = 0;
    gl[7] = 0;
    gu[7] = 0;
    // object that computes objective and constraints

    // options
    string strOptions;
    // turn off any printing
    strOptions += "Integer print_level  0\n";
    strOptions += "String sb            yes\n";
    // maximum iterations
    strOptions += "Integer max_iter     100\n";
    // approximate accuracy in first order necessary conditions;
    //  see Mathematical Programming, Volume 106, Number 1,
    //  Pages 25-57, Equation (6)
    strOptions += "Numeric tol          1e-6\n";
    // derivative tesing
    strOptions += "String derivative_test   second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    strOptions += "Numeric point_perturbation_radius   0.\n";

    CppAD::ipopt::solve_result<Dvector> solution;  // solution
    // solve the problem
    CppAD::ipopt::solve<Dvector, CFGEval>(strOptions, p0, xl, xu, gl, gu, m_iFGEval, solution);

    vector<AD<mdouble>> vResX =
        m_iFGEval.MappingP2X((solution.x)[0], (solution.x)[1], (solution.x)[2], (solution.x)[3], (solution.x)[4]);

    mdouble dS = CppAD::Value(vResX[4]);
    mdouble dStep = 0.05;
    vector<mdouble> vdXDiscrete, vdYDiscrete, vdSpiralPathX, vdSpiralPathY, vdSpiralPathYaw, vdSpiralPathCurvr, vdSSet;
    vector<CPose2D> vSpiralPath;
    for (mdouble i = 0; i < dS + dStep; i += dStep)
    {
        mdouble theta_yaw =
            CppAD::Value(m_iFGEval.CalTheta(iStartPose.m_dRz, vResX[0], vResX[1], vResX[2], vResX[3], i));
        vdXDiscrete.push_back(cos(theta_yaw));
        vdYDiscrete.push_back(sin(theta_yaw));
        vdSpiralPathYaw.push_back(theta_yaw);
        vdSSet.push_back(i);
        vdSpiralPathCurvr.push_back(
            1.0 / max(fabs(CppAD::Value(m_iFGEval.CalCurv(vResX[0], vResX[1], vResX[2], vResX[3], i))),
                      0.01));  // Avoid infinite curvature
    }
    vdSpiralPathX = CumulativeTrapezoid(vdXDiscrete, vdSSet, iStartPose.m_dx);
    vdSpiralPathY = CumulativeTrapezoid(vdYDiscrete, vdSSet, iStartPose.m_dy);

    for (muint i = 0; i < vdSpiralPathX.size(); i++)
    {
        CPose2D iTmpPose;
        iTmpPose.m_dx = vdSpiralPathX[i];
        iTmpPose.m_dy = vdSpiralPathY[i];
        iTmpPose.m_dRz = vdSpiralPathYaw[i];
        vSpiralPath.push_back(iTmpPose);
    }

    mdouble dMinR = *min_element(vdSpiralPathCurvr.begin(), vdSpiralPathCurvr.end());

    mbool bResFlag;
    mbool bDistanceCost = dS < m_dDistanceCostCoef * sqrt(pow(iStartPose.m_dx - iGoalPose.m_dx, 2) +
                                                          pow(iStartPose.m_dy - iGoalPose.m_dy, 2));
    mbool bGoalCost =
        sqrt(pow(vdSpiralPathX.back() - iGoalPose.m_dx, 2) + pow(vdSpiralPathY.back() - iGoalPose.m_dy, 2) +
             pow(vdSpiralPathYaw.back() - iGoalPose.m_dRz, 2)) < m_dGoalCostTolerence;

    if (bDistanceCost && bGoalCost)
    {
        bResFlag = true;
    }
    else
    {
        bResFlag = false;
    }

    return make_tuple(vSpiralPath, dMinR, bResFlag);
}
