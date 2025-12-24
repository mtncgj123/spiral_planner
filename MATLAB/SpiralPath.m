clear;close all;clc;

addpath('/home/iasl/git/LogAnalyzer/tool/navigation');
%%根据起始位姿和目标位姿计算螺线轨迹
% 输入起始位姿和目标位姿
matStartPose =[125.6 91.8807 pi];  
matGoalPose=[123.274 91.278 -1.571];


matNormalizedPose = NormalizePose(matStartPose,matGoalPose);

dExpectMinR=1;
dExpectMaxCurv=1/dExpectMinR;

dDistance=norm([matStartPose(1)-matGoalPose(1) matStartPose(2)-matGoalPose(2)]);
nGear = CalcGear(matStartPose,matGoalPose);

% 3. 用匿名函数封装参数（仅把x作为输入，参数a/b固定为当前值）
% fun = @(x) nonlin_eq(x, matStartPose,matGoalPose);  
fun = @(x) nonlin_eq(x, [0 0 0],matNormalizedPose); 

% 4. 调用fsolve求解（传入封装后的匿名函数）
x0 = [0,0,dDistance*nGear];  
options = optimoptions('fsolve', 'Display', 'iter');  
[x, fval, exitflag, output] = fsolve(fun, x0, options);

% 输出结果
disp('求解结果：');
disp(['x1 = ', num2str(x(1)), ', x2 = ', num2str(x(2)),', x3 = ', num2str(x(3))]);
disp(['残差平方和：', num2str(sum(fval.^2))]);
disp(['退出标志：', num2str(exitflag)]);

[dA0,dA1,dA2,dA3,dS1]=Mapping_k2a(0,x(1),x(2),0,x(3));
[matSpiralPose,dMinCurvR]=CalcDiscretePath(matStartPose,dS1,[dA0 dA1 dA2 dA3]);


subplot(2,1,1);
plot(matSpiralPose(:,1), matSpiralPose(:,2),'LineWidth',2);
xlabel('x(m)') 
ylabel('y(m)')
axis equal;

subplot(2,1,2);
hold on;
plot(matSpiralPose(:,5), matSpiralPose(:,4),'LineWidth',2);
yline(dExpectMaxCurv, '--', 'Color','red', 'LineWidth', 3);
yline(-dExpectMaxCurv, '--', 'Color','red', 'LineWidth', 3);
xlabel('s(m)') 
ylabel('curv')
axis equal;
