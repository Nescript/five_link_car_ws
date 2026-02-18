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

### 2.3 MATLAB 辅助推导

