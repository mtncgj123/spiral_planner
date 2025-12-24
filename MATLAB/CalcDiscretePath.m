%% 根据起始位姿，总长度，多项式参数A计算离散的螺线轨迹 
%% 输入参数：
%   matStartPose: 起始位姿
%   dS: 总长度
%   vParamA: 多项式参数A
%% 输出参数：
%   matSpiralPose: 离散的螺旋线轨迹 x,y,yaw,curv,s
%   dMinCurvR: 最小转弯半径
%% 调用说明：

function [matSpiralPose,dMinCurvR]=CalcDiscretePath(matStartPose,dS,vParamA)
dStep=0.01;
dMaxCurv=0;
nSign=sign(dS);
vDiscreteX=[];
vDiscreteY=[];
vDiscreteS=[];
vSpiralPathYaw=[];
vSpiral_path_curv=[];
dA0=vParamA(1);
dA1=vParamA(2);
dA2=vParamA(3);
dA3=vParamA(4);
dStartPoseX=matStartPose(1);
dStartPoseY=matStartPose(2);
dStartPoseYaw=matStartPose(3);
for i=0:dStep:abs(dS)
    dDiscreteS=i*nSign;
    dYaw=CalcTheta(dStartPoseYaw,dA0,dA1,dA2,dA3,dDiscreteS);
    vDiscreteX=[vDiscreteX;cos(dYaw)];
    vDiscreteY=[vDiscreteY;sin(dYaw)];
    vSpiralPathYaw=[vSpiralPathYaw;dYaw];
    vDiscreteS=[vDiscreteS;dDiscreteS];
    dCurv=CalcCurv(dA0,dA1,dA2,dA3,dDiscreteS);
    vSpiral_path_curv=[vSpiral_path_curv;dCurv];
    if abs(dCurv)>dMaxCurv
        dMaxCurv=abs(dCurv);
    end
end
vSpiral_path_x=cumulative_trapezoid(vDiscreteX,vDiscreteS,dStartPoseX);
vSpiral_path_y=cumulative_trapezoid(vDiscreteY,vDiscreteS,dStartPoseY);
matSpiralPose=[vSpiral_path_x vSpiral_path_y vSpiralPathYaw,vSpiral_path_curv vDiscreteS];
dMinCurvR=1/dMaxCurv;
