function [c, ceq] = nonlin_con(x)
    % 非线性不等式约束：c(x) ≤ 0 （需将原约束转换为 ≤0 形式）
    % 原约束 x1² + x2² ≤ 4 → c = x1² + x2² - 4 ≤ 0
    c =[];  
    
    % 非线性等式约束：ceq(x) = 0
    % 原约束 x1 + x2 = 1 → ceq = x1 + x2 - 1 = 0
    ceq = [];  
end