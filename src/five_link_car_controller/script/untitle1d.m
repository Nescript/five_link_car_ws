syms x_b y_b x_d y_d real
syms phi_2 phi_3 real
syms l2 l3 real
eq1 = x_d + l3 * cos(phi_3) == x_b + l2 * cos(phi_2)
eq2 = y_b + l2 * sin(phi_2) == y_d + l3 * sin(phi_3)
eq3 = l3^2 == (x_b - x_d + l2 * cos(phi_2))^2 + (y_b - y_d + l2 * sin(phi_2))^2
sol = solve(eq3, phi_2, 'Real', true)
