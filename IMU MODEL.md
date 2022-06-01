# IMU模型

## 1.基本模型

**IMU误差模型**

**误差分类**

- 加速度计和陀螺仪的误差可以分为：==确定性误差==、==随机误差==
- 确定性误差可以事先标定确定，包括：bais、scale
- 随机误差通常假设噪声服从高斯分布，包括：高斯白噪声、bais随机游走...

忽略scale，只考虑高斯白噪声n和bais随机游走b：


$$
\begin{aligned}
    {\tilde{\mathbf{\omega}}^b} &= {\mathbf{\omega}^b} + {\mathbf{b}^g} + { \mathbf{n}^g } \\
    {\tilde{\mathbf{a}}^b} &= {\mathbf{q}_{bw}} ( {\mathbf{a}^w} + {\mathbf{g}^w}) + {\mathbf{b}^a} + {\mathbf{n}^a}
\end{aligned}
$$

有上标波浪线的代表的是陀螺仪的测量值  $ {\tilde{\mathbf{\omega}}^b}$  和加速度计的测量值  ${\tilde{\mathbf{a}}^b}$
右上标 $b$ 表示的是 body坐标系，$w$ 表示世界坐标系或惯性坐标系，$a$ 表示加速度acc, $g$ 表示gyro陀螺仪。

所以位移 $p$，速度 $v$，四元数 $q$ 的微分形式如下所示，${\otimes}$ 代表四元数之间的乘法。

$$
\begin{aligned}
    {\dot{\mathbf{p}}_{wb_t}} &= {\mathbf{v}^w_t} \\
    {\dot{\mathbf{v}}^w_t} &=  {\mathbf{a}^w_t}\\
    {\dot{\mathbf{q}}_{wb_t}} &= {\mathbf{q}_{wb_t}} {\otimes} \begin{bmatrix}
                                                    0\\
                                                    {\frac{1}{2}}{\mathbf{\omega}}^{b_t}\\
                                                    \end{bmatrix}
\end{aligned}
$$


## 2.连续时间下IMU运动模型
根据上面的微分形式，可以从第 $i$ 个时刻的位移 ${\mathbf{p}_{wb_i}}$，速度 ${\mathbf{v}^w_i}$，四元数 ${\mathbf{q}_{wb_i}}$ 通过IMU的测量值进行积分，得到第 $j$ 时刻的位移（$ {\mathbf{p}_{wb_j}}$）、速度（$ {\mathbf{v}^w_j}$）、四元数（${\mathbf{q}_{wb_j}}$）。

$$
\begin{aligned}
    {\mathbf{p}_{wb_j}} &= {\mathbf{p}_{wb_i}} + {{\mathbf{v}}^w_t}{{\triangle}t} + {\iint_{t{\in}[i,j]}} ( {\mathbf{q}_{wb_t}} {\mathbf{a}^{b_t}} - \mathbf{g}^w ) {\delta}t^2  \\
    {\mathbf{v}^w_j} &= {\mathbf{v}^w_i} + {\int_{t{\in}[i,j]}} ( {\mathbf{q}_{wb_t}} {\mathbf{a}^{b_t}} - \mathbf{g}^w ) {\delta}t  \\
    {\mathbf{q}_{wb_j}} &= {\int_{t{\in}[i,j]}}{\mathbf{q}_{wb_t}} {\otimes} \begin{bmatrix}
                                                    0\\
                                                    {\frac{1}{2}}{\mathbf{\omega}}^{b_t}  \\
                                                    \end{bmatrix}{\delta}t
\end{aligned}
$$

## 3.运动模型的离散积分--欧拉法
使用欧拉法，即两个相邻时刻 $k$ 到 $k+1$ 的位姿是用第$k$时刻的测量值 $\mathbf{a}$，${\mathbf{\omega}}$ 来计算的

$$
\begin{aligned}
    {\mathbf{p}_{wb_{k+1}}} &= {\mathbf{p}_{wb_k}} + {{\mathbf{v}}^w_t}{{\triangle}t} +  {\frac{1}{2}\mathbf{a}{{\triangle}t^2}}  \\
    {\mathbf{v}^w_{k+1}} &= {\mathbf{v}^w_k} + {\mathbf{a}{{\triangle}t}}  \\
    {\mathbf{q}_{wb_{k+1}}} &= {\mathbf{q}_{wb_{k}}} {\otimes} \begin{bmatrix}
                                                    0\\
                                                    {\frac{1}{2}}{\mathbf{\omega}{\delta}t}\\
                                                    \end{bmatrix}
\end{aligned}
$$

其中，

$$
\begin{aligned}
    {\mathbf{a}} &= {\mathbf{q}_{wb_k}} ( \mathbf{a}^{b_k} - \mathbf{b}^a_k ) - {\mathbf{g}^w}  \\
    {\mathbf{\omega}} &= {\mathbf{\omega}^{b_k}} - {\mathbf{b}^g_k}
\end{aligned}
$$

## 4.运动模型的离散积分--中值积分法

$$
\begin{aligned}
    {\mathbf{p}_{wb_{k+1}}} &= {\mathbf{p}_{wb_k}} + {{\mathbf{v}}^w_t}{{\triangle}t} +  {\frac{1}{2}\mathbf{a}{{\triangle}t^2}}  \\
    {\mathbf{v}^w_{k+1}} &= {\mathbf{v}^w_k} + {\mathbf{a}{{\triangle}t}}  \\
    {\mathbf{q}_{wb_{k+1}}} &= {\mathbf{q}_{wb_{k}}} {\otimes} \begin{bmatrix}
                                                    0\\
                                                    {\frac{1}{2}}{\mathbf{\omega}{\delta}t}\\
                                                    \end{bmatrix}
\end{aligned}
$$

其中，

$$
\begin{aligned}
    {\mathbf{a}} &= {\frac{1}{2}}[{\mathbf{q}_{wb_k}} ( \mathbf{a}^{b_k} - \mathbf{b}^a_k ) - {\mathbf{g}^w} + {\mathbf{q}_{wb_{k+1}}} ( \mathbf{a}^{b_{k+1}} - \mathbf{b}^a_k ) - {\mathbf{g}^w}]  \\
    {\mathbf{\omega}} &= {\frac{1}{2}}[{\mathbf{\omega}^{b_k}} - {\mathbf{b}^g_k} + {\mathbf{\omega}^{b_{k+1}}} - {\mathbf{b}^g_k}]
\end{aligned}
$$

