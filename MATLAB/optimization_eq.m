%% 生成最优化代价函数
%% 输入参数：
%   x: 需要求解的未知数，三等分点的曲率K1,K2 总长度S
%   matStartPose: 起始位姿
%   matGoalPose: 目标位姿
%% 输出参数：
%   F: 最优化代价函数
%% 调用说明：

function F = optimization_eq(x, matStartPose,matGoalPose)
    dK0=0;
    dK1 = x(1);
    dK2 = x(2);
    dK3=x(3);
    dS=x(4);
    dStartPoseX=matStartPose(1);
    dStartPoseY=matStartPose(2);
    dStartPoseYaw=matStartPose(3);
    dGoalPoseX=matGoalPose(1);
    dGoalPoseY=matGoalPose(2);
    dGoalPoseYaw=matGoalPose(3);
    dGoalCostCoef=100;
    [dA0,dA1,dA2,dA3,dS1]=Mapping_k2a(dK0,dK1,dK2,dK3,dS);
    dF1 = CalcPoseX(dStartPoseX,dStartPoseYaw,dA0,dA1,dA2,dA3,dS1)-dGoalPoseX; 
    dF2 = CalcPoseY(dStartPoseY,dStartPoseYaw,dA0,dA1,dA2,dA3,dS1)-dGoalPoseY;
    dF3 = CalcTheta(dStartPoseYaw,dA0,dA1,dA2,dA3,dS1)-dGoalPoseYaw;
    F=dGoalCostCoef*(dF1*dF1+dF2*dF2+dF3*dF3)+CalcCurvSumCost(dA0,dA1,dA2,dA3,dS1);
end