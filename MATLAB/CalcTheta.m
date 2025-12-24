%% 计算角度
%% 输入参数：
%   dStartYaw: 初始角度
%   dA0: 0阶系数
%   dA1: 1阶系数
%   dA2：2阶系数
%   dA3: 3阶系数
%   dS：总长度
%% 输出参数：
%   dYaw：计算得到的角度
%% 调用说明：

function dYaw=CalcTheta(dStartYaw,dA0,dA1,dA2,dA3,dS)
dYaw=dStartYaw+dA3*dS*dS*dS*dS/4+dA2*dS*dS*dS/3+dA1*dS*dS/2+dA0*dS;
end