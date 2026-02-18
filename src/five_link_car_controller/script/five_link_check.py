import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class FiveBarKinematics:
    def __init__(self):
        # 设定杆长 (单位：m)
        self.l1 = 0.14  # 主动臂 L1, L4
        self.l2 = 0.25  # 从动臂 L2, L3
        self.l5 = 0.12  # 基座间距
        
        # 基座坐标
        self.xA, self.yA = 0, 0
        self.xD, self.yD = self.l5, 0

    def inverse_kinematics(self, xc, yc):
        """
        求解主动臂角度 phi1 和 phi4 (弧度)
        """
        # --- 左侧支链 (A-B-C) ---
        dist_sq_left = xc**2 + yc**2
        d_left = np.sqrt(dist_sq_left)
        
        # 增加极小偏移量防止 arccos 在边界处因浮点误差报错
        val_left = (self.l1**2 + dist_sq_left - self.l2**2) / (2 * self.l1 * d_left)
        if not (-1.0001 <= val_left <= 1.0001): return None
        
        gamma_left = np.arctan2(yc, xc)
        alpha_left = np.arccos(np.clip(val_left, -1, 1))
        phi1 = gamma_left + alpha_left 

        # --- 右侧支链 (D-E-C) ---
        dx_right = xc - self.l5
        dist_sq_right = dx_right**2 + yc**2
        d_right = np.sqrt(dist_sq_right)
        
        val_right = (self.l1**2 + dist_sq_right - self.l2**2) / (2 * self.l1 * d_right)
        if not (-1.0001 <= val_right <= 1.0001): return None
            
        gamma_right = np.arctan2(yc, dx_right)
        alpha_right = np.arccos(np.clip(val_right, -1, 1))
        phi4 = gamma_right - alpha_right 
        
        return phi1, phi4

# --- 1. 计算工作空间并提取边界 ---
robot = FiveBarKinematics()
res = 200 
x_range = np.linspace(-0.2, 0.35, res)
y_range = np.linspace(-0.1, 0.4, res)
X, Y = np.meshgrid(x_range, y_range)

d_left = np.sqrt(X**2 + Y**2)
d_right = np.sqrt((X - robot.l5)**2 + Y**2)
r_min, r_max = abs(robot.l1 - robot.l2), robot.l1 + robot.l2

workspace_mask = (d_left >= r_min) & (d_left <= r_max) & \
                 (d_right >= r_min) & (d_right <= r_max)

# 绘图初始化
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')
ax.grid(True, linestyle=':', alpha=0.6)

# 绘制背景轮廓
ax.contourf(X, Y, workspace_mask, levels=[0.5, 1], colors=['#E8F5E9'])
cs = ax.contour(X, Y, workspace_mask, levels=[0.5], colors=['#4CAF50'], linewidths=1)

# --- 2. 提取边界作为轨迹 ---
# 从等高线对象中提取路径顶点
paths = cs.collections[0].get_paths()
if paths:
    # 选取最长的路径（通常是主工作区边界）
    main_path = max(paths, key=lambda p: len(p.vertices))
    boundary_coords = main_path.vertices
    x_traj = boundary_coords[:, 0]
    y_traj = boundary_coords[:, 1]
else:
    # 如果没提取到则使用演示轨迹
    t = np.linspace(0, 2*np.pi, 100)
    x_traj, y_traj = 0.06+0.05*np.cos(t), 0.2+0.05*np.sin(t)

# --- 3. 预计算动画坐标 ---
history_coords = []
for xi, yi in zip(x_traj, y_traj):
    res_ik = robot.inverse_kinematics(xi, yi)
    if res_ik:
        p1, p4 = res_ik
        xb, yb = robot.l1 * np.cos(p1), robot.l1 * np.sin(p1)
        xe, ye = robot.l5 + robot.l1 * np.cos(p4), robot.l1 * np.sin(p4)
        history_coords.append(((0, xb, xi, xe, robot.l5), (0, yb, yi, ye, 0)))

# --- 4. 动画执行 ---
line, = ax.plot([], [], 'b-o', lw=3, markersize=6, label='Limit State')
ax.plot([0, robot.l5], [0, 0], 'ks', markersize=10)

def update(frame):
    xs, ys = history_coords[frame]
    line.set_data(xs, ys)
    return line,

ani = FuncAnimation(fig, update, frames=len(history_coords), interval=30, blit=True)

plt.title("5-Bar Linkage Moving Along Workspace Boundary")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.show()