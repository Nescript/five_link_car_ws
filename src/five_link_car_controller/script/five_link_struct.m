syms l1 l2 l3 l4 l5 real
syms alpha beta theta1 theta2 real

% 1. 定义位置约束方程 f = [fx; fy] = 0
% 左链末端减去右链末端
fx = l1*cos(alpha) + l2*cos(theta1) - (l5 + l4*cos(beta) + l3*cos(theta2));
fy = l1*sin(alpha) + l2*sin(theta1) - (l4*sin(beta) + l3*sin(theta2));
f = [fx; fy];

% 2. 提取驱动变量 q 和从动变量 th
q = [alpha; beta];
th = [theta1; theta2];

% 3. 计算偏导数矩阵 (隐函数求导准备)
fq = jacobian(f, q);  % ∂f/∂q
fth = jacobian(f, th); % ∂f/∂th

% 4. 求从动角对驱动角的导数矩阵 (d_theta / d_q)
% 根据隐函数求导法则：d_th/dq = -inv(fth) * fq
dth_dq = -inv(fth) * fq;

% 5. 定义末端 B 点的位置 (使用左链表示)
xB = l1*cos(alpha) + l2*cos(theta1);
yB = l1*sin(alpha) + l2*sin(theta1);
B = [xB; yB];

% 6. 计算末端对所有变量的偏导
Bxq = jacobian(B, q);  % ∂B/∂q
Bxth = jacobian(B, th); % ∂B/∂th

% 7. 合成最终雅可比矩阵 J
% 根据全微分：dB = (∂B/∂q)dq + (∂B/∂th)dth = (∂B/∂q + ∂B/∂th * dth/dq)dq
J = Bxq + Bxth * dth_dq;

% 8. 简化结果
J = simplify(J);

disp('位置全微分法得到的雅可比矩阵 J:');
disp(J);