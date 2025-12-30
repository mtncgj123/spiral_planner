clear;close all;clc;
addpath('/home/iasl/git/LogAnalyzer/tool/basic_math');
addpath('/home/iasl/git/LogAnalyzer/tool/navigation');
%%根据起始位姿和目标位姿计算螺线轨迹
% 输入起始位姿和目标位姿
matStartPose =[117.704 94.6062 pi/2];  
matGoalPose=[118.544 96.2257 1.576274];


matNormalizedPose = NormalizePose(matStartPose,matGoalPose);

dExpectMinR=1;
dExpectMaxCurv=1/dExpectMinR;

dDistance=norm([matStartPose(1)-matGoalPose(1) matStartPose(2)-matGoalPose(2)]);
nGear = CalcGear(matStartPose,matGoalPose);

%% 求解非线性方程组
% 3. 用匿名函数封装参数（仅把x作为输入，参数a/b固定为当前值） 
% fun = @(x) nonlin_eq(x, [0 0 0],matNormalizedPose); clc
% 
% 4. 调用fsolve求解（传入封装后的匿名函数）
% x0 = [0,0,dDistance*nGear];  
% options = optimoptions('fsolve', 'Display', 'iter');  
% [x, fval, exitflag, output] = fsolve(fun, x0, options);
% 
% 输出结果
% disp('求解结果：');
% disp(['x1 = ', num2str(x(1)), ', x2 = ', num2str(x(2)),', x3 = ', num2str(x(3))]);
% disp(['残差平方和：', num2str(sum(fval.^2))]);
% disp(['退出标志：', num2str(exitflag)]);
% 
% [dA0,dA1,dA2,dA3,dS1]=Mapping_k2a(0,x(1),x(2),0,x(3));
% [matSpiralPose,dMinCurvR]=CalcDiscretePath(matStartPose,dS1,[dA0 dA1 dA2 dA3]);
% fprintf('Max cruve: %6.3f\n',1/dMinCurvR);

%% 求解非线性最优化
% fun = @(x) optimization_eq(x, [0 0 0],matNormalizedPose); 
% 
% % 2. 设置初始值（如x0 = [0, 0]）
% x0 = [0,0,0,dDistance*nGear]; 
% 
% % 3. 调用fminunc（可选：设置优化选项，如显示迭代信息、使用梯度）
% options = optimoptions('fminunc', 'Display', 'iter', 'Algorithm', 'quasi-newton');
% [x_opt, f_opt, exitflag, output] = fminunc(fun, x0, options);
% 
% % 4. 输出结果
% disp('最优解 x_opt：');
% disp(x_opt);
% disp('最优目标函数值 f_opt：');
% disp(f_opt);
% 
% [dA0,dA1,dA2,dA3,dS1]=Mapping_k2a(0,x_opt(1),x_opt(2),x_opt(3),x_opt(4));
% [matSpiralPose,dMinCurvR]=CalcDiscretePath(matStartPose,dS1,[dA0 dA1 dA2 dA3]);
% fprintf('Max cruve: %6.3f\n',1/dMinCurvR);

%% 调用fminco求解非线性最优化
fun = @(x) optimization_eq(x, [0 0 0],matNormalizedPose); 

% 2. 设置初始值（如x0 = [0, 0]）
x0 = [0,0,0,dDistance*nGear];  

% 2. 线性约束（无则设为[]）
A = []; b = [];
Aeq = []; beq = [];

% 3. 变量上下界：x1 ≥ 0, x2 ≥ 0
lb = [-100, -100];  
ub = [100, 100];  

% 4. 优化选项（可选，如显示迭代信息、设置求解器）
options = optimoptions('fmincon', ...clc
    'Display', 'iter', ...  % 显示迭代过程
    'Algorithm', 'interior-point');  % 内点法（默认，适合大多数问题）

% 5. 调用fmincon求解
[x_opt, fval, exitflag, output] = fmincon(fun, x0, A, b, Aeq, beq, lb, ub,@nonlin_con, options);

% 6. 输出结果
disp('最优解 x：');
disp(x_opt);
disp('最优目标函数值 fval：');
disp(fval);
disp('退出标志 exitflag（>0表示收敛）：');
disp(exitflag);

[dA0,dA1,dA2,dA3,dS1]=Mapping_k2a(0,x_opt(1),x_opt(2),x_opt(3),x_opt(4));
[matSpiralPose,dMinCurvR]=CalcDiscretePath(matStartPose,dS1,[dA0 dA1 dA2 dA3]);
fprintf('Max cruve: %6.3f\n',1/dMinCurvR);


%% 结果后处理
% 将路径角度转到-pi-pi
for i=1:size(matSpiralPose,1)
    matSpiralPose(i,3)= convert_angle(matSpiralPose(i,3));
end

%% 绘制计算结果

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
yline(1/dMinCurvR, '--', 'Color','blue', 'LineWidth', 1);
yline(-1/dMinCurvR, '--', 'Color','blue', 'LineWidth', 1);
xlabel('s(m)') 
ylabel('curv')
axis equal;
