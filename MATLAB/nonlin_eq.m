%% 根据约束生成非线性方程组
%% 输入参数：
%   x: 需要求解的未知数，三等分点的曲率K1,K2 总长度S
%   matStartPose: 起始位姿
%   matGoalPose: 目标位姿
%% 输出参数：
%   F: 非线性方程组 终点位姿约束
%% 调用说明：

function F = nonlin_eq(x, matStartPose,matGoalPose)
    dK0=0;
    dK1 = x(1);
    dK2 = x(2);
    dK3=0;
    dS=x(3);
    dStartPoseX=matStartPose(1);
    dStartPoseY=matStartPose(2);
    dStartPoseYaw=matStartPose(3);
    dGoalPoseX=matGoalPose(1);
    dGoalPoseY=matGoalPose(2);
    dGoalPoseYaw=matGoalPose(3);
    [dA0,dA1,dA2,dA3,dS1]=Mapping_k2a(dK0,dK1,dK2,dK3,dS);
    F(1) = CalcPoseX(dStartPoseX,dStartPoseYaw,dA0,dA1,dA2,dA3,dS1)-dGoalPoseX; 
    F(2) = CalcPoseY(dStartPoseY,dStartPoseYaw,dA0,dA1,dA2,dA3,dS1)-dGoalPoseY;
    F(3) = CalcTheta(dStartPoseYaw,dA0,dA1,dA2,dA3,dS1)-dGoalPoseYaw;
end