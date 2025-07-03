# 三维旋转实现

在计算机图形学中，旋转是物体最基本的运动状态之一。本文将探讨计算机图形学中表示三维旋转的主要方法，
包括欧拉角、旋转矩阵和四元数。

___

## 齐次坐标 (Homogeneous Coordinates)
有意思的是，这个和本文的重点关系不大，但我们还是要着重讲一下，因为齐次坐标为表示三维空间中的变换（旋转、平移、缩放）提供了统一的框架，理解齐次坐标对于更广泛的变换场景至关重要。

在标准三维空间中，旋转和缩放都是线性变换，因而可以直接使用3x3矩阵表示（此处仅展示缩放矩阵，旋转矩阵将在稍后展示）:
$$
\begin{bmatrix}
x' \\  
y' \\
z'
\end{bmatrix}
=
\begin{bmatrix}
s_x & 0 & 0 \\
0 & s_y & 0 \\
0 & 0 & s_z
\end{bmatrix}
\begin{bmatrix}
x \\
y \\
z
\end{bmatrix}
$$
但是平移变换却不是线性的，因而你需要额外加一个位移向量才能继续使用3x3矩阵：
$$
\begin{bmatrix}
x' \\  
y' \\
z'
\end{bmatrix}
=
\begin{bmatrix}
a & b & c \\
d & e & f \\
h & i & j
\end{bmatrix}
\begin{bmatrix}
x \\
y \\
z
\end{bmatrix} +
\begin{bmatrix}
t_x \\
t_y \\
t_z
\end{bmatrix}
$$
通过齐次坐标，一个三维点表示为 $(\mathbf{x}, \mathbf{y}, \mathbf{z}, 1)^\mathbf{T}$，一个三维向量表示为 $(\mathbf{x}, \mathbf{y}, \mathbf{z}, 0)$。这允许将所有变换（包括平移）统一用 4x4 矩阵表示,
之前的公式就可以替换为:

$$
\begin{bmatrix}x' \\
y' \\ 
z' \\
1 \end{bmatrix}=\begin{bmatrix}a&b&c&t_x \\
d&e&f&t_y \\ 
h&i&j&t_z \\
0&0&0&1\end{bmatrix}\begin{bmatrix}x \\
y \\
z \\
1\end{bmatrix}
$$

为简洁起见，后续部分将专注于旋转公式，不使用齐次坐标。
___

##  欧拉角(Euler Angle)

**欧拉角**  使用三个角度$\alpha、\beta、\gamma$分别表示绕 X、Y、Z 轴的旋转。它们直观易懂.但存一个致命的问题——万向节锁（Gimbal Lock），
即当绕第二个轴旋转 90 度时，其他两个旋转轴会重合，导致失去一个旋转自由度。

欧拉角的万向节锁的矩阵证明如下（此处使用Z-Y-X的变换顺序）:


__首先，我们需要三个绕三个轴旋转的矩阵:__

$$
R_x(\alpha) = \begin{bmatrix} 1 & 0 & 0 \\
0 & \cos(\alpha) & -\sin(\alpha) \\
0 & \sin(\alpha) & \cos(\alpha) \end{bmatrix} 
$$

$$ 
R_y(\beta) = \begin{bmatrix} \cos(\beta) & 0 & \sin(\beta) \\
0 & 1 & 0 \\
-\sin(\beta) & 0 & \cos(\beta) \end{bmatrix}
$$

$$
R_z(\gamma) = \begin{bmatrix} \cos(\gamma) & -\sin(\gamma) & 0 \\
\sin(\gamma) & \cos(\gamma) & 0 \\
0 & 0 & 1 \end{bmatrix}
$$

__此外，还需要一点点三角函数的变换:__

$$ 
\begin{aligned}&\sin(\alpha - \beta) = \sin(\alpha)\cos(\beta) - \cos(\alpha)\sin(\beta) \\
&\cos(\alpha - \beta) = \cos(\alpha)\cos(\beta) + \sin(\alpha)\sin(\beta)\end{aligned}
$$

__于是，我们得到:__

$$ 
\begin{aligned} 
& R_z(\gamma)R_y(\frac{\pi}{2})R_x(\alpha) \\
& = \begin{bmatrix} \cos(\gamma) & -\sin(\gamma) & 0 \\
\sin(\gamma) & \cos(\gamma) & 0 \\ 
0 & 0 & 1 \end{bmatrix}\begin{bmatrix} 0 & 0 & 1 \\
0 & 1 & 0 \\
-1 & 0 & 0 \end{bmatrix}\begin{bmatrix} 1 & 0 & 0 \\
0 & \cos(\alpha) & -\sin(\alpha) \\ 
0 & \sin(\alpha) & \cos(\alpha) \end{bmatrix}\\
& = \begin{bmatrix} 0 & \cos(\gamma)\sin(\alpha)-\cos(\alpha)\sin(\gamma) & \sin(\alpha)\sin(\gamma)+\cos(\alpha)\cos(\gamma)\\
0 & \sin(\alpha)\sin(\gamma)+\cos(\alpha)\cos(\gamma) & \cos(\alpha)\sin(\gamma)-\cos(\gamma)\sin(\alpha)\\
-1 & 0 & 0 \end{bmatrix} \\
& = \begin{bmatrix} 0 & \sin(\alpha - \gamma) & \cos(\alpha - \gamma) \\ 
0 & \cos (\alpha - \gamma) & -\sin(\alpha - \gamma) \\
-1 & 0 & 0 \end{bmatrix} \\
& = R_y(\frac{\pi}{2})R_x(\alpha - \gamma)
\end{aligned}
$$

我们发现$R_z$在最终表达式中消失了，这就表明失去了一个自由度。单这么说有点抽象，我们来具体分析一下。因为欧拉角原本是用于描述空间中的姿态而非旋转。
事实上，“旋转”就是一连串离散的姿态在经过插值之后以一定的帧率播放得到的视觉上的连续的过程。
而如果你将欧拉角变换过程视为一个函数
$$
f(x, y, z) \rightarrow TargetAspect
$$
你会发现这个函数很明显是一个满射函数（欧拉证明了），但是它却不是单射的。这就意味着同一个姿态并不会对应唯一的变换，
观察上面的公式你也会发现$(\alpha , \pi/2, \gamma)$和$(-\gamma, \pi/2, -\alpha)$可以表示相同的姿态，而对应的旋转轴在空间中是共线的。
当“旋转”到$\beta=\pi/2$时，对$\alpha$和$\gamma$的操作将导向共线的结果，这就阐明了自由度丢失的问题了。

事实上，刚刚证明的是基于本地坐标系的欧拉角，还有一种基于全局坐标系的欧拉角，不过当表示复杂的姿态变换时，全局欧拉角就难堪一用了。

---

## 旋转矩阵（Rodrigues' Rotation Formula）

罗德里格斯旋转公式通过旋转角度$\theta$ 和旋转轴 $\vec{r}$ 直接表示旋转。

$$
K = \begin{bmatrix}0&-r_z&r_y \\
r_z&0&-r_x \\
-r_y&r_x&0\end{bmatrix} 
$$

$$
R(\theta,\vec{r}) = I + \sin(\theta) K + (1 - \cos\theta)K^2
$$

展开后得到:

$$ 
R(\theta,\vec{r}) = 
\begin{bmatrix}\cos\theta+r_x^2(1-\cos\theta)&r_xr_y(1-\cos\theta)-r_z\sin\theta&r_xr_z(1-\cos\theta)+r_y\sin\theta \\
r_yr_x(1-\cos\theta)+r_z\sin\theta&\cos\theta+r_y^2(1-\cos\theta)&r_yr_z(1-\cos\theta)-r_x\sin\theta \\
r_zr_x(1-\cos\theta)-r_y\sin\theta&r_zr_y(1-\cos\theta)+r_x\sin\theta&\cos\theta+r_z^2(1-\cos\theta)\end{bmatrix} 
$$

它的优势在于可以避免万向节锁，而且适用于单轴旋转的直接表示。缺点也很明显，它的存储需要 9 个数字(对比之下四元数只需要4个)；
而且在连续的变化之后会累计一定的精度误差，这可能会使矩阵失去正交性，因为这种误差最终会体现剪切形变，需要定期正交化修正；
最后我们在前文中提到过旋转需要一定的插值操作（毕竟你做3D动画肯定不能一帧一帧做），
罗德里格斯旋转矩阵的复杂性也使它难以实现平滑插值
___
## 四元数（Quaternions）
四元数为旋转提供了一种紧凑且数值稳定的表示方法，特别适用于动画和三维仿真等需要平滑插值和避免万向锁的场景。

一个四元数可以表示为 $q = w + xi + yj + zk$\
当实部为0时，这个纯四元数可以表示一个向量

这是一些四元数的基本运算:
- __共轭: $q^* = w - xi - yj - zk$__
- __点积t: $p \cdot q= w_1w_2 + x_1x_2 + y_1y_2 + z_1z_2$__
- __外积: $p\times q=(w_1w_2-x_1x_2-y_1y_2-z_1z_2)+(w_1x_2+x_1w_2+y_1z_2-y_2z_1)i+(w_1y_2-x_1z_2+y_1w_2+z_1x_2)j+(w_1z_2=z_1x_2-y_1x_2+z_1w_2)k$__
- __求逆: $q^{-1} = \frac{q^*}{q \cdot q}$__

__基于罗德里格斯旋转矩阵，我们可以这样表示旋转:__

$$
Q = \cos\left(\frac{\theta}{2}\right) + \sin\left(\frac{\theta}{2}\right)(u_x i + u_y j + u_z k) 
$$

$$ 
X' = QXQ^{-1} 
$$

__其中 $(u_x, u_y, u_z)$ 表示旋转轴 $\theta$ 表示旋转角__\
__一个四元数也可以表示为矩阵:__

$$
w+xi+yj+zk=\begin{bmatrix}w^2+x^2-y^2-z^2&2xy-2wz&2xz+2wy\\
2xy+2wz&w^2-x^2+y^2-z^2&2yz-2wx\\
2xz-2wy&2yz+2wx&w^2-x^2-y^2+z^2\end{bmatrix} 
$$

### 插值操作
1. 线性插值 (Lerp):
$$
Lerp(p, q, t) = (1-t)p+tq 
$$
   - 优点：简单、快速.
   - 缺点：输出未归一化，可能影响稳定性；旋转速度变化明显
   - 适用场景：小角度旋转或需要极高效率的场景.
2. 归一化线性插值(Nlerp):
$$ 
Nlerp(p,q,t) = \frac{(1-t)p+tq}{||(1-t)p+tq||}
$$

  - 优点：计算快速，输出归一化。
  - 缺点：旋转速度非恒定。
  - 适用场景：实时应用中优先考虑计算效率的场景。
3. 球面线性插值 (Slerp):
$$ 
Slerp(p, q, t) = \frac{\sin[(1 - t)\theta]p + \sin(t\theta)q}{\sin\theta}
$$
- 优点：沿单位四元数球的最短路径插值，保持恒定旋转速度，适合平滑过渡。
- 缺点：当点积为负时，Slerp 可能走较长路径（需对其中一个四元数取负）；当 (\theta \approx 0) 时，(\sin\theta) 接近 0，可能导致数值不稳定，需改用线性插值。
- 适用场景：动画、游戏或仿真中需要高精度和平滑性的场景。
4. 球面四边形样条插值 （Spherical and Quadrangle Spline Interpolation，Squad）:
$$ 
Squad(p_0,p_1,p_2,p_3,t) = Slerp(Slerp(p_0,p_1,t),Slerp(p_2,p_3,t),2t(1-t))
$$

   -优点：使用四个控制点定义插值曲线形状，支持平滑的二阶连续性，适合复杂动画中的连续运动。
   - 缺点：计算复杂。
   - 适用场景：需要多点平滑过渡的动画或机器人应用。

由于单位四元数天然具有正交性，于是罗德里格斯旋转矩阵的问题得到了解决，那四元数的缺点是什么呢？四元数无法表示平移或缩放，仍需齐次坐标来处理这些变换。
