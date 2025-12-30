%% 计算曲率积分代价
%% 输入参数：
%   dA0: 0阶系数
%   dA1: 1阶系数
%   dA2：2阶系数
%   dA3: 3阶系数
%   dAS：总长度
%% 输出参数：
%   dCurvSumCost：计算得到的曲率积分代价
%% 调用说明：

function dCurvSumCost=CalcCurvSumCost(dA0,dA1,dA2,dA3,dS)
dCurvSumCost=(dA3*dA3/7*dS^7+1/3*dA2*dA3*dS^6+1/5*(2*dA1*dA3+dA2*dA2)*dS^5+1/2*(dA3*dA0+dA2*dA1)*dS^4+1/3*(dA1*dA1+2*dA0*dA2)*dS^3+dA0*dA1*dS*dS+dA0*dA0*dS)*dS/abs(dS); 
end