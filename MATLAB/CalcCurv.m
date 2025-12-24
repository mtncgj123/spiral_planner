%% 计算曲率
%% 输入参数：
%   dA0: 0阶系数
%   dA1: 1阶系数
%   dA2：2阶系数
%   dA3: 3阶系数
%   dAS：总长度
%% 输出参数：
%   dCurv：计算得到的曲率
%% 调用说明：

function dCurv=CalcCurv(dA0,dA1,dA2,dA3,dS)
dCurv=dA3*dS*dS*dS+dA2*dS*dS+dA1*dS+dA0;
end