%% 将三等分点的曲率映射到多项式参数A
%% 输入参数：
%   dK0: 起始点曲率
%   dK1: 三等分点曲率
%   dK2: 三等分点曲率
%   dK3: 终点曲率
%   dS：总长度
%% 输出参数：
%   dA0: 0阶系数
%   dA1: 1阶系数
%   dA2：2阶系数
%   dA3: 3阶系数
%   dAS1：总长度
%% 调用说明：

function [dA0,dA1,dA2,dA3,dS1]=Mapping_k2a(dK0,dK1,dK2,dK3,dS)
dA0=dK0;
dA1=-(11/2*dK0-9*dK1+9/2*dK2-dK3)/dS;
dA2=(9*dK0-45/2*dK1+18*dK2-9/2*dK3)/dS/dS;
dA3=-(9/2*dK0-27/2*dK1+27/2*dK2-9/2*dK3)/dS/dS/dS;
dS1=dS;
end