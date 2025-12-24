%% 根据梯形法则计算累计积分
%% 输入参数：
%   vDiscrete_function_value: 函数离散值
%   vX_set: 自变量离散值
%   dInitial: 积分起始值
%% 输出参数：
%   vCumulative_res: 累计积分结果
%% 调用说明：

function vCumulative_res=cumulative_trapezoid(vDiscrete_function_value,vX_set,dInitial)
vCumulative_res=[];
dSum=dInitial;
vCumulative_res=[vCumulative_res;dSum];
for i=1:length(vX_set)-1
    dSum=dSum+(vDiscrete_function_value(i)+vDiscrete_function_value(i+1))/2*(vX_set(i+1)-vX_set(i));
    vCumulative_res=[vCumulative_res;dSum];
end