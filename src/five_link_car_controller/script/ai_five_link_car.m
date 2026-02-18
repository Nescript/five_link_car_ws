% filepath: derivation.m
syms t m_wheel m_P M R I_wheel I_P I_M L L_M l g T T_P
syms x(t) theta(t) phi(t) % 定义为随时间变化的函数

% 1. 定义基本坐标关系
x_b = x(t) + (L + L_M) * sin(theta(t));
y_b = (L + L_M) * cos(theta(t)) + l * cos(phi(t));

pos_P_x = x(t) + L * sin(theta(t));
pos_P_y = L * cos(theta(t));

pos_M_x = x_b - l * sin(phi(t));
pos_M_y = y_b; % 注意：机体质心高度由y_b决定

% 2. 计算加速度（统一求导）
acc_x = diff(x, t, 2);
acc_P_x = diff(pos_P_x, t, 2);
acc_P_y = diff(pos_P_y, t, 2);
acc_M_x = diff(pos_M_x, t, 2);
acc_M_y = diff(pos_M_y, t, 2);

% 3. 列出力学方程 (引入相互作用力作为未知数)
syms N N_f P N_M P_M ddx ddtheta ddphi
% 轮组
Eq1 = N_f - N == m_wheel * acc_x;
Eq2 = I_wheel * (acc_x / R) == T - N_f * R;
% 摆杆
Eq3 = N - N_M == m_P * acc_P_x;
Eq4 = P - P_M - m_P * g == m_P * acc_P_y;
Eq5 = I_P * diff(theta, t, 2) == (P * L + P_M * L_M) * sin(theta) - (N * L + N_M * L_M) * cos(theta) - T + T_P;
% 机体
Eq6 = N_M == M * acc_M_x;
Eq7 = P_M - M * g == M * acc_M_y;
Eq8 = I_M * diff(phi, t, 2) == T_P + N_M * l * cos(phi) + P_M * l * sin(phi);

% 4. 替换二阶导项为符号变量以进行代数求解
old_vars = [diff(x,t,2), diff(theta,t,2), diff(phi,t,2)];
new_vars = [ddx, ddtheta, ddphi];
AllEqs = subs([Eq1, Eq2, Eq3, Eq4, Eq5, Eq6, Eq7, Eq8], old_vars, new_vars);

% 5. 联立求解 (包含相互作用力 N, N_f, P, N_M, P_M 和加速度)
sol = solve(AllEqs, [ddx, ddtheta, ddphi, N, N_f, P, N_M, P_M]);

% 6. 提取加速度结果
ddx_final = simplify(sol.ddx)
ddtheta_final = simplify(sol.ddtheta)
ddphi_final = simplify(sol.ddphi)