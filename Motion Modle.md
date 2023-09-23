# Quadrupedal Motional Model
## 机械狗运动模型（Motion Model）描述
机器人与环境接触的机械系统的运动模型描述方程可以描述如下：
$$
{\bf M}({\bf q}){\bf \dot u} + {\bf h}({\bf q}, {\bf u}) =  {\bf S}^T{\bf \tau} + {\bf J}_S^T{\bf \lambda}
\tag {1}$$
这其中${\bf q}$是一个描述机器人主体及各个节点的广义位置矢量：
$$
{\bf q}= \begin{bmatrix}_I {\bf r}_{IB} \\ {\bf q}_{IB} \\ {\bf q}_j\end{bmatrix} \in SE(3)\times {\Bbb R}^{n_j}
\tag {1:1}$$
它里面$_I {\bf r}_{IB} \in {\Bbb R}^{3}$是机器人主体相对于惯性系的三维位置矢量；${\bf q}_{IB} \in SO(3)$是机器人主体相对于惯性系的转动描述，用哈密顿单位四元数表示的；${\bf q}_j \in {\Bbb R}^{n_j}$是一个储存机器人所有节点角度的$n_j$维矢量。
不过我还是不太清楚$\bf q$也就是$SE(3)\times {\Bbb R}^{n_j}$到底是一个什么形状的矩阵？单纯看表达的话他应该是一个$3+4+n_j$维的矢量，包含了整个机器人的所有位置有关的状态信息。它似乎不用直接参与运算，因此可以先放一放，重要的是$SE(3)\times {\Bbb R}^{n_j}$的含义到底是什么需要好好理解一下。
这其中${\bf u}$是一个描述机器人主体及各个节点的广义速度矢量：
$$
{\bf u}= \begin{bmatrix}_I{\bf v}_B \\ _B{\bf \omega}_{IB} \\ {\bf \dot q}_j \end{bmatrix} \in {\Bbb R}^{n_u} 
\tag {1:2}$$
它里面的$_I{\bf v}_B$描述了机器人主体相对于惯性系的速度；$_B{\bf \omega}_{IB}$描述了机器人主体相对于自身的角速度；${\bf \dot q}_j$描述了机器人的各个关节转动的速度。
这其中${\bf M}$是一个关于机器人整体的质量矩阵，它是一个$n_u \times n_u$的矩阵： 
$${\bf M} \in {\Bbb R}^{n_u \times n_u} \tag{1:3}$$
这个质量矩阵的具体数值跟机器人的机械系统的状态（各个节点的位置${\bf q}$）相关，可以通过通用的方式计算出它的表达式。实际计算的时候只需要带入${\bf q}$的值，就可以计算出${\bf M}$矩阵的各个元素具体数值。
这其中${\bf h}$是一个跟机器人的机械位置和速度都有关的量，包含了机械系统产生的克里奥利力、离心力和重力的作用，它是一个$n_u$维的矢量：
$${\bf h} \in {\Bbb R}^{n_u} \tag{1:4}$$
它的具体计算方式我现在还不是很清楚。
这其中${\bf S}$是一个选择矩阵，可以用来选择整个公式中哪些自由度被激活，它是一个$n_{\tau} \times n_u$的矩阵：
$${\bf S} = \begin{bmatrix}{\bf 0}_{n_{\tau} \times (n_u - n_{\tau})} & {\Bbb I}_{n_{\tau} \times n_{\tau}}\end{bmatrix} \in {\Bbb R}^{n_{\tau} \times n_u} 
\tag{1:5}$$
它其中包含的参数$n_{\tau}$表示被激活的自由度数量，如果机器人的所有自由度都被激活，则$n_{\tau} = n_j$。
这其中${\bf \tau}$是机器人各个关节的电机提供的扭矩，它是一个$n_j$维的向量：
$${\bf \tau} \in {\Bbb R}^{n_j} 
\tag{1:6}$$
它的节点成员是否产生作用受到${\bf S}^T$矩阵的选择。
这其中${\bf J}_S$是一些列雅可比矩阵的集合：
$$J_S=\begin{bmatrix}J^T_{C_1} & ... & J^T_{C_{n_c}}\end{bmatrix}^T  \in {\Bbb R}^{3n_c\times n_u}$$ 
它是接触点的支撑力${\bf \lambda}$向节点力转换的矩雅可比矩阵，包含了$n_c$个雅可比矩阵，$n_c$为接触地面的肢体个数。

## 质心运动(CoM Motion)问题表述
每一个坐标方向的质心运动规划都被描述成一个系列的五次样条。比如沿着x方向的其中第$i-th$曲线可以描述为：
$$\begin{align}
x(t) = & a_{i5}^x t^5 +  a_{i4}^x t^4 + a_{i3}^x t^3 +  a_{i2}^x t^52+  a_{i1}^x t^1 +  a_{i0}^x t^0 \\
= & \begin{bmatrix}t^5 & t^4 & t^3 & t^2 & t^1 & t^0\end{bmatrix} \\ & \cdot \begin{bmatrix}a_{i5} & a_{i4} & a_{i3} & a_{i2} & a_{i1} & a_{i0}\end{bmatrix} \\
= & {\bf \eta}^T(t){\bf \alpha}_i^x
\end{align}\tag{2}$$
这其中$t\in [t, t+\Delta t_i]$是第$i$个曲线段前所有$(i-1)$个曲线持续时间长度的总和，$\Delta t_i$是第$i$个曲线持续的时长。
基于$(2)$的关于位置的表述，可以很容易地将关于速度和加速度的表述写出来：
$$\begin{align}
\dot x(t) = {\bf \dot \eta}^T(t){\bf \alpha}_i^x \\
\ddot x(t) = {\bf \ddot \eta}^T(t) {\bf \alpha}_i^x 
\tag{3}\end{align}$$
这其中：
$$\begin{align}
{\bf \dot \eta}^T(t) =  \begin{bmatrix}5t^4 & 4t^3 & 3t^2 & 2t^1 & 1 & 0\end{bmatrix}^T\\
{\bf \ddot \eta}^T(t) =  \begin{bmatrix}20t^3 & 12t^2 & 6t^t & 2 & 0 & 0\end{bmatrix}^T
\tag{4}\end{align}$$
对于质心在$y,\, z$方向上的描述是一样的。每一条质心曲线都由$x,y,z$三部分分量${\bf \alpha}_i = \begin{bmatrix}{{\bf \alpha}_i^x}^T & {{\bf \alpha}_i^y}^T & {{\bf \alpha}_i^z}^T\end{bmatrix}^T$组成，可以将$n_s$条质心曲线的$3n_s$条曲线参数写到一起，优化的参数矢量可以写成：${\bf \alpha} = \begin{bmatrix}{\bf \alpha}_0^T & ...& {\bf \alpha}_i^T &...& {\bf \alpha}_{n_s}^T\end{bmatrix}^T$。
这样一来，质心的位置可以表示为：
$$
{\bf p}_{CoM}(t) = {\bf T}(t){\bf \alpha}_i \in{\Bbb R}^3
, \quad
{\bf T}(t) = 
\begin{bmatrix}
{\bf \eta}^T(t) & 0 & 0 \\
0 & {\bf \eta}^T(t) & 0 \\
0 & 0 & {\bf \eta}^T(t)
\end{bmatrix}\tag{5}$$
同样质心的速度和位置也可以得到了：
$$\begin{align}
{\bf \dot p}_{CoM}(t) = {\bf \dot T}(t){\bf \alpha}_i \in{\Bbb R}^3 \\
{\bf \ddot p}_{CoM}(t) = {\bf \ddot T}(t){\bf \alpha}_i \in{\Bbb R}^3
\end {align}$$

## 质心运动优化
