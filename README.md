# 五连杆学习
## 1. 概览
目前 RM 赛场上的的轮腿主要分为并联腿和串并联混合两种。为[降低腿部惯量，提高总质心](https://zhuanlan.zhihu.com/p/683709573)，这两种结构都将驱动电机放置在机体上，通过**连杆结构驱动腿部关节**。少见真正意义上的串联腿（电机通常直接置于各个关节上）。

这两种轮腿构型，都涉及连杆结构的解算。以并联腿为例，我们通过调整两髋关节电机角度以控制连杆末端位置。

## 2. 公式推导
### 2.1 运动学正解
五连杆的运动学正解即给定髋关节电机角度，计算末端位置坐标，如下图所示：
![alt text](/img/image.png)

不妨令点 A 为坐标原点，我们可以借助极坐标系写出点 B 和点 D 的坐标表达式：
$$
\begin{equation}
\begin{cases}
B = (l_1 \cos\phi_1, l_1 \sin\phi_1) \\
D = (l_5 + l_4 \cos\phi_4, l_4 \sin\phi_4)
\end{cases}
\end{equation}
$$
解算时，上述表达式中所有量均为已知，可直接代入计算。

进一步可以分别从点 B，点 D 出发，写出点 C 的坐标表达式：
$$
\begin{equation}
\begin{cases}
C = B + (l_2 \cos\phi_2, l_2 \sin\phi_2) \\
C = D + (l_3 \cos\phi_3, l_3 \sin\phi_3)
\end{cases}
\end{equation}
$$

该表达式可写为：
$$
\begin{equation}
\begin{cases}
x_b + l_2 \cos\phi_2 = x_d + l_3 \cos\phi_3 \\
y_b + l_2 \sin\phi_2 = y_d + l_3 \sin\phi_3
\end{cases}
\end{equation}
$$

其中的 $\phi_2$ 和 $\phi_3$ 未知，但若能求出其中一者关于 B，D 坐标的表达式，则可进一步写出 C 点坐标。
接下来以求 $\phi_2$ 表达式为例进行推导。

消去 $\phi_3$ 的过程如下：
由 (3) 
$$
\begin{equation}
\begin{cases}
(x_b - x_d) + l_2 \cos\phi_2 =  l_3 \cos\phi_3 \\
(y_b - y_d) + l_2 \sin\phi_2 = l_3 \sin\phi_3
\end{cases}
\end{equation}
$$
由 (4) 上下平方后相加得
$$
\begin{equation}
(x_b - x_d + l_2 \cos\phi_2)^2 + (y_b - y_d + l_2 \sin\phi_2)^2 = l_3^2
\end{equation}
$$

分别将 $x_b - x_d$ 和 $y_b - y_d$ 视为整体，使用二项式完全平方公式有
$$
\begin{equation}
(x_b - x_d)^2 + (y_b - y_d)^2 + 2(x_b - x_d)l_2 \cos\phi_2 + 2(y_b - y_d)l_2 \sin\phi_2 + l_2^2 = l_3^2
\end{equation}
$$
其中的 $(x_b - x_d)^2 + (y_b - y_d)^2$ 即为点 B 与点 D 间距离的平方，即 $l_{BD}^2$，故有：
$$
\begin{equation}
2(x_b - x_d)l_2 \cos\phi_2 + 2(y_b - y_d)l_2 \sin\phi_2= l_3^2 - l_2^2 - l_{BD}^2
\end{equation}
$$
此时 $\phi_2$ 还在 $\cos(\phi_2)$ 和 $\sin(\phi_2)$ 中，还不太好求。可设：
$$
\begin{equation}
\begin{cases}
a = 2(x_b - x_d)l_2 \\
b = 2(y_b - y_d)l_2 \\
c = l_3^2 - l_2^2 - l_{BD}^2
\end{cases}
\end{equation}
$$
则原式变为
$$
\begin{equation}
a \cos\phi_2 + b \sin\phi_2 = c
\end{equation}
$$
进一步可使用万能公式：
$$
\begin{equation}
\begin{cases}
\sin(\alpha) = \frac{2\tan(\frac{\alpha}{2})}{1 + \tan^2(\frac{\alpha}{2})} \\
\cos(\alpha) = \frac{1 - \tan^2(\frac{\alpha}{2})}{1 + \tan^2(\frac{\alpha}{2})}
\end{cases}
\end{equation}
$$
得
$$
\begin{equation}
a \cdot \frac{1 - \tan^2(\frac{\phi_2}{2})}{1 + \tan^2(\frac{\phi_2}{2})} + b \cdot \frac{2\tan(\frac{\phi_2}{2})}{1 + \tan^2(\frac{\phi_2}{2})} = c
\end{equation}
$$
至此，原式变为了关于 $\tan(\frac{\phi_2}{2})$ 的一元二次方程，令 $\tan(\frac{\phi_2}{2})$ 为 $z$，则有：
$$
(a + c)z^2 - 2bz + (c - a) = 0
$$
那么综上所述，我们就得到了 $\phi_2$ 的表达式:
$$
\begin{equation}
\phi_2 = 2 \arctan(\frac{b \pm \sqrt{a^2 + b^2 - c^2}}{a+c}) \newline

\end{equation}
$$
其中：
$$
\begin{cases}
a = 2(x_b - x_d)l_2 \\
b = 2(y_b - y_d)l_2 \\
c = l_3^2 - l_2^2 - l_{BD}^2 \\
l_{BD} = \sqrt{(x_b - x_d)^2 + (y_b - y_d)^2}
\end{cases}
$$

### 2.2 逆解
逆解即给定末端坐标，计算髋关节电机角度

我们知点 C 的坐标 $(x_c, y_c)$ 可分别通过 B 点与 D 点坐标配合 $\phi_2$ 和 $\phi_3$ 求得，即由
$$
\begin{equation}
\begin{cases}
x_c = l_1 \cos\phi_1 + l_2 \cos\phi_2 \\
y_c = l_1 \sin\phi_1 + l_2 \sin\phi_2
\end{cases}
\end{equation}
$$
我们试图寻找的是 $\phi_1$ 和 $\phi_4$ 和 $x_c, y_c$ 的关系，故此处的 $\phi_2$ 是干扰项，通过上下同时平方求和消去得：
$$
\begin{equation}
(x_c - l_1 \cos\phi_1)^2 + (y_c - l_1 \sin\phi_1)^2 = l_2^2
\end{equation}
$$
展开得
$$
\begin{equation}
2x_c l_1 \cos\phi_1 + 2y_c l_1 \sin\phi_1 = x_c^2 + y_c^2 + l_1^2  - l_2^2
\end{equation}
$$
令
$$
\begin{cases}
a = 2x_c l_1 \\
b = 2y_c l_1 \\
c = x_c^2 + y_c^2 + l_1^2 - l_2^2
\end{cases}
$$
得到
$$
\begin{equation}
a \cos\phi_1 + b \sin\phi_1 = c
\end{equation}
$$
再次利用万能公式将原式转化为关于 $\tan(\frac{\phi_1}{2})$ 的二元一次方程，最终解算结果为：
$$ 
\phi_1 = 2 \arctan(\frac{b \pm \sqrt{a^2 + b^2 - c^2}}{a+c}) \newline
$$
$\phi_4$ 可同理算出。

根据前文的代数推导方法，$\phi_4$ 的推导过程如下：

同理，点 C 的坐标 $(x_c, y_c)$ 也可以通过右侧基座点（坐标为 $(l_5, 0)$）、右侧曲柄 $l_4$ 和连杆 $l_3$ 来表示：
$$
\begin{equation}
\begin{cases}
x_c = l_5 + l_4 \cos\phi_4 + l_3 \cos\phi_3 \\
y_c = l_4 \sin\phi_4 + l_3 \sin\phi_3
\end{cases}
\end{equation}
$$

为了求出 $\phi_4$，我们需要消去干扰项 $\phi_3$。将含有 $\phi_4$ 的项移到等式左边：
$$
\begin{equation}
\begin{cases}
(x_c - l_5) - l_4 \cos\phi_4 = l_3 \cos\phi_3 \\
y_c - l_4 \sin\phi_4 = l_3 \sin\phi_3
\end{cases}
\end{equation}
$$

将两式上下同时平方并求和，利用 $\sin^2\phi_3 + \cos^2\phi_3 = 1$ 消去 $\phi_3$：
$$
\begin{equation}
((x_c - l_5) - l_4 \cos\phi_4)^2 + (y_c - l_4 \sin\phi_4)^2 = l_3^2
\end{equation}
$$

将等式左边展开：
$$
\begin{equation}
(x_c - l_5)^2 - 2(x_c - l_5)l_4 \cos\phi_4 + l_4^2 \cos^2\phi_4 + y_c^2 - 2y_c l_4 \sin\phi_4 + l_4^2 \sin^2\phi_4 = l_3^2
\end{equation}
$$

提取公因式并整理，将含有 $\phi_4$ 的项移到等式左边，常数项移到等式右边：
$$
\begin{equation}
2(x_c - l_5)l_4 \cos\phi_4 + 2y_c l_4 \sin\phi_4 = (x_c - l_5)^2 + y_c^2 + l_4^2 - l_3^2
\end{equation}
$$

此时，等式形式与 $\phi_1$ 的推导完全一致。我们令：
$$
\begin{cases}
a' = 2(x_c - l_5)l_4 \\
b' = 2y_c l_4 \\
c' = (x_c - l_5)^2 + y_c^2 + l_4^2 - l_3^2
\end{cases}
$$

得到标准形式：
$$
\begin{equation}
a' \cos\phi_4 + b' \sin\phi_4 = c'
\end{equation}
$$

再次利用万能公式将原式转化为关于 $\tan(\frac{\phi_4}{2})$ 的一元二次方程，最终解算结果为：
$$ 
\phi_4 = 2 \arctan\left(\frac{b' \pm \sqrt{a'^2 + b'^2 - c'^2}}{a'+c'}\right) \newline
$$

*(注：公式中的 $\pm$ 对应了右腿膝盖向左弯曲或向右弯曲的两种不同装配姿态)*

### 2.3 MATLAB 辅助推导
```
This is the code block that represents the suggested code change:
```markdown
// ...existing code...
$$ 
\phi_4 = 2 \arctan\left(\frac{b' \pm \sqrt{a'^2 + b'^2 - c'^2}}{a'+c'}\right) \newline
$$

*(注：公式中的 $\pm$ 对应了右腿膝盖向左弯曲或向右弯曲的两种不同装配姿态)*

### 2.3 几何法推导（代码实现方案）
在实际代码实现（如 `five_link_car_controller.cpp`）中，为了计算效率和逻辑清晰，我们通常采用**几何法**。

#### 2.3.1 正解几何法（双圆交点）
正解问题可转化为：已知两圆圆心和半径，求两圆交点。
1. **计算膝关节坐标**：
   已知电机角度 $\theta_1, \theta_4$，可直接得左右膝关节点 B 和 D 的坐标：
   - 左膝 $B$: $(l_1 \cos\phi_1, l_1 \sin\phi_1)$
   - 右膝 $E$ (代码中对应右腿肘部): $(l_5 + l_4 \cos\phi_4, l_4 \sin\phi_4)$
   *(注：代码中 $\phi_1 = \theta_1 + \pi$，$\phi_4 = \theta_4$)*

2. **化简为双圆交点问题**：
   末端点 C (代码中为 P) 既在以 B 为圆心、$l_2$ 为半径的圆上，也在以 E 为圆心、$l_3$ 为半径的圆上。
   令 $d = |BE|$ 为两圆心距离。若满足三角形不等式 $|l_2 - l_3| \le d \le l_2 + l_3$，则两圆相交。

3. **求解交点**：
   设向量 $\vec{BE}$ 方向为局部 x 轴，先求交点在局部坐标系下的垂足距离 $a$ 和垂线高度 $h$：
   由余弦定理或几何关系：
   $$ a = \frac{l_2^2 - l_3^2 + d^2}{2d} $$
   $$ h = \sqrt{l_2^2 - a^2} $$
   
   基准点 M (垂足) 坐标为：
   $$ M = B + \frac{a}{d}(E - B) $$
   
   则交点 C 的坐标为：
   $$ 
   \begin{cases}
   x_c = x_m \pm \frac{h}{d}(y_e - y_b) \\
   y_c = y_m \mp \frac{h}{d}(x_e - x_b)
   \end{cases}
   $$
   通常取 $y$ 较大的解（对应膝盖向上的构型）。

#### 2.3.2 逆解几何法（三角形余弦定理）
逆解问题可拆分为左右两个独立的三角形问题求解。

1. **左侧支链 (A-B-C)**：
   已知 A(0,0), C($x_c, y_c$)，边长 $l_1, l_2$。
   - 距离 $L_{AC} = \sqrt{x_c^2 + y_c^2}$
   - 这三条边构成三角形，由余弦定理求角 $\alpha$ ( $\angle BAC$ 相关的辅助角)：
     $$ \cos\alpha = \frac{l_1^2 + L_{AC}^2 - l_2^2}{2 l_1 L_{AC}} $$
   - 向量 $\vec{AC}$ 的基础角度 $\gamma = \operatorname{atan2}(y_c, x_c)$
   - 最终驱动角 $\phi_1 = \gamma + \alpha$ (对应肘部外拐构型)

2. **右侧支链 (D-E-C)**：
   已知右基座 D($l_5, 0$), C($x_c, y_c$)，边长 $l_4, l_3$。
   - 右基座到末端向量 $\vec{DC} = (x_c - l_5, y_c)$，距离 $L_{DC} = |\vec{DC}|$
   - 同理由余弦定理求角 $\beta$：
     $$ \cos\beta = \frac{l_4^2 + L_{DC}^2 - l_3^2}{2 l_4 L_{DC}} $$
   - 向量 $\vec{DC}$ 的基础角度 $\gamma_r = \operatorname{atan2}(y_c, x_c - l_5)$
   - 最终驱动角 $\phi_4 = \gamma_r - \beta$ (对应肘部内拐构型)

### 2.4 MATLAB 辅助推导

