# Quadrupedal Motional Model
## 0. 机械狗运动模型（Motion Model）描述
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

## 1. 质心运动(CoM Motion)问题表述
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

## 2. 质心运动优化
机械狗的质心运动天然受到多种条件约束。因此，我们将质心运动规划问题描述为一个非线性优化问题，它的目标是在相等约束${\bf c}({\bf \xi})$和不等约束${\bf h}({\bf \xi})$的条件下最小化一个通用的成本函数${\bf f}({\bf \xi})$。解决这个问题的数值方法被称作为顺序二次规划算法((SQP) algorithm)，这个算法要求计算约束条件和成本函数的雅可比和海森矩阵。这个优化算法会以局部运动的踱步周期$\tau$为时间间隔，不断地重新计算新的结果。
下面的内容将介绍一些用到的成本函数和约束条件：

### a. _成本函数：最小化运动规划结果的加速度。_ 
通过计算样条段的二次成本可以得到：
$$
{\bf Q}_k^{acc} = 
\begin{bmatrix}
(400/7)\Delta t_k^7 & 40\Delta t_k^6 & 24\Delta t_k^5 & 10\Delta t_k^4 & \\
40\Delta t_k^6 & 28.8\Delta t_k^5 & 18\Delta t_k^4 & 8\Delta t_k^3 & \\
24\Delta t_k^5 & 18\Delta t_k^4 & 12\Delta t_k^3 & 6\Delta t_k^2 & {\bf 0}_{4\times 2}\\
10\Delta t_k^4 & 8\Delta t_k^3 & 6\Delta t_k^2 & 4\Delta t_k &\\
 & {\bf 0}_{2\times 4} & & & {\bf 0}_{2\times 2}
\end{bmatrix}
\tag{6}$$
相应的${\bf c}_k^{acc}=0$。$\Delta t_k$是第$k$个曲线段以秒为单位的持续时间。
${\bf Q}_k^{acc}$是通过平方和积分CoM在第$k$样条的持续时间上的加速度得到。${\bf Q}_k^{acc}$被添加为整个成本函数的Hessian的子矩阵。（${\bf Q}_k^{acc}$到底是成本函数还是Hessian矩阵？它们之间有什么关系？有必要了解一下Hessian矩阵去。）
最终期望的位置计算为参考高级线性速度$v_{ref}$和角速度$\omega_{ref}$的$z$分量和优化时间间隔$\tau$的函数。在以上参数相对于优化时间间隔$\tau$变化很缓慢的情况下，最后的位置可以计算为：
$$
{\bf p}_{final} = {\bf p}_{init} + {\bf R}(\tau {\bf \hat \omega}_z)(\tau {\bf v}_{ref})
\tag{7}$$
这里面第一项${\bf p}_{init}$是前一时刻的位置，第二项是当前时刻的位移$\tau {\bf v}_{ref}$乘上一个旋转矩阵${\bf R}(\tau {\bf \hat \omega}_z)=\begin{bmatrix}0 & 0 & {\bf \omega}_{ref_z}\end{bmatrix}^T$。
路径正则化${\bf  \pi}$。$\bf \pi$的初始位置计算为优化视界$\tau$上平均的足迹中心，假设脚要么接触，要么以恒定的速度移动。最终位置是通过参考高级速度计算的，如参考文献（7）所示。

我们将初始速度设置为参考速度$v_{ref}$ ，最后一个速度与运动计划结束时预测的躯干方向对齐。${\bf  \pi}$的初始和最终加速度设置为零。${\bf  \pi}$的初始和最终高度设置为用户指定的参考值。（这个${\bf  \pi}$到底是个啥？运动规划的高级计算近似结果，比如用传感器得到的轨迹推算出来的结果？）
我们通过添加以下形式的成本$\lambda_{lin}\epsilon_z+\lambda_{quad}\epsilon_z^2$来限制沿 z 轴超调。同时对原有描述扩充以下几个约束：
$$\begin{align}
{\bf p}_{CoM}^z(t)-{\bf \pi}_z(t)\leq &\epsilon_z \\
{\bf p}_{CoM}^z(t)-{\bf \pi}_z(t)\geq & -\epsilon_z \\
\epsilon_z \geq & 0
\end{align}\tag{8}$$
这里面$\epsilon$是一个松弛变量，它会被添加入优化参数${\bf \xi}$中。由于时间依赖性，需要对轨迹进行采样，其中每个采样点引入两个额外的不等式约束。（为什么？）

### b. _等式约束：运动曲线段节点连续性。_
对于$x$方向上的第$k$段和第$k+1$段曲线的连续性要求可以写作：
$$
\begin{bmatrix}{\bf \eta}(t_{fk})^T & -{\bf \eta}(0)^T \\ {\bf \dot \eta}(t_{fk})^T & -{\bf \dot \eta}(0)^T\end{bmatrix}
\begin{bmatrix}{\bf \alpha}_k^x \\ {\bf \alpha}_{k+1}^x\end{bmatrix}
=0
\tag{9}$$
这里面$t_{fk}$代表曲线${\bf s}_k$以秒为单位的持续时间长度。同样的方法，可以将$y,z$方向上的连续性要求写出来。
节点连续性在涉及到悬空自由落体过程时需要被另外对待：起飞时的质心的动力学可以描述为：${\bf \ddot [}_{CoM} = {\bf g}$，其中${\bf g}$是重力加速度矢量。将上式积分可以得到：
$$\begin{align}
{\bf \dot p}(t) =& {\bf g}t+{\bf \dot p}_{CoM}(0) \\
{\bf p}(t) =& \frac{1}{2}{\bf g}t^2+{\bf \dot p}_{CoM}(t)+{\bf p}_{CoM}(0)(0) 
\end{align}\tag{10}$$
着陆时的质心动力学可以描述为：
$$\begin{align}
{\bf \ddot T}(0){\bf \alpha}_{i+1} =& {\bf g}\\
{\bf \dot T}(0){\bf \alpha}_{i+1} =& {\bf g}t_f + {\bf \dot T}(t_i){\bf \alpha}_{i} \\
{\bf T}(0){\bf \alpha}_{i+1} =& \frac{1}{2}{\bf g}t_f^2+ [{\bf \dot T}(t_i)t_f+{\bf T}(t_i)]{\bf \alpha}_{i} \\
\end{align}\tag{10}$$
这里面$t_i$表示第$i$条曲线段的持续时间。如果第一个或最后一个支持多边形对应于一个完整的飞行阶段，则可以在初始和最终条件下找到类似的替换。（这句话的含义是什么？）

### c. _不等约束：基于ZMP._
>这部分我还是看不太懂，这家伙已经不用购物车模型来近似计算ZMP了，还要再去看看ZMP的知识。
#### (1) ZMP的计算
$$
{\bf p}_{ZMP} = \frac{{\bf n}\times {\bf M}_O^{gi}}{{\bf n}^T {\bf F}^{gi}}
\tag{12}$$
这里面${\bf M}_O^{gi}$和${\bf F}^{gi}$是重力-惯性力螺旋的组成部分，它们的计算方式如下：
$$\begin{align}
{\bf M}_O^{gi} = & m\cdot {\bf p}_{CoM}\times ({\bf g}-{\bf \ddot p}_{CoM})-{\bf \dot L} \\
{\bf F}^{gi} = & m\cdot{\bf g} - {\bf \dot P}
\end{align}\tag{13}$$
这里面$m, {\bf P}, {\bf L}$分别是质心的质量、线性动量和角动量。因为我们将不会针对转动及其延伸进行优化，因此下面的计算中角动量近似为零${\bf \dot L}=0$。
#### (2) 引入的不等约束
基本要求是ZMP点药被约束在机械狗的支撑多边形内部，这给出以下形式的不等约束：
$$
{\bf h}_{ZMP} = {\bf d}^T{\bf p}_{ZMP}+r \geq 0
\tag{14}$$
这其中${\bf d}^T = \begin{bmatrix}p&q&0\end{bmatrix}$和$r$是描述支撑多边形的边的系数。接下来将$(12)$代入$(14)$后可以得到：
$$\begin{align}&{\bf d}^T{\bf S}({\bf n}){\bf M}_O^{gi} + r{\bf n}^T{\bf F}^{gi} \\
= &{\bf d}^T{\bf S}({\bf n})[m\cdot {\bf p}_{CoM}\times ({\bf g}-{\bf \ddot p}_{CoM})-{\bf \dot L}] + r{\bf n}^T(m\cdot{\bf g} - {\bf \dot P}) \\
\overset{{\bf \dot L}\approx 0 | {\bf \dot P}=m{\bf \ddot p}_{CoM}}{=} & m{\bf d}^T{\bf S}({\bf n}){\bf p}_{CoM}\times ({\bf g}-{\bf \ddot p}_{CoM})+ mr{\bf n}^T({\bf g}-{\bf \ddot p}_{CoM}) \geq 0 \\
\rightarrow & {\bf d}^T{\bf S}({\bf n}){\bf S}({\bf p}_{CoM})({\bf g}-{\bf \ddot p}_{CoM})+ r{\bf n}^T({\bf g}-{\bf \ddot p}_{CoM}) \geq 0\end{align}\tag{15}$$
这里面定义了一个斜对称矩阵${\bf S}(a)$用来实现将矩阵叉乘转化为点乘的效果（这个方法优惠什么标准依据？）。它通过计算一个可以实现以下效果的方程得到： ${\bf S}({\bf a}){\bf b} = {\bf a}\times {\bf b}$。
用$(15)$也可以计算出$(14)$相对于质心位置${\bf p}_{CoM}$和质心加速度${\bf \ddot p}_{CoM}$的梯度（为什么要求这个梯度？怎么求得？）：
$$
\nabla{\bf h}_{ZMP} = 
\begin{bmatrix}
\Gamma\cdot ({\bf \ddot p}_{CoM}-{\bf g}) \\
-\Gamma\cdot {\bf p}_{CoM}-r{\bf n}
\end{bmatrix}
\tag{16}$$
这里面定义${\bf \Gamma}={\bf S}({\bf S}^T({\bf n}){\bf d}$。同样，它的Hessian 矩阵计算为：
$$
\nabla^2{\bf h}_{ZMP} = 
\begin{bmatrix}
0 & {\bf \Gamma}^T \\
-{\bf \Gamma}^T & 0
\end{bmatrix}
\tag{17}$$
这个Hessian矩阵是一个反对称矩阵，因此可以从优化问题中去除。（为什么呢？Hessian矩阵怎么计算的也得去了解一下。）
> 这部分不是很明白。为什么对问题描述添加那样的项就可以软化不等约束？

我们软化了初始$n_{ineq}$个样本的不等式约束，其中$n_{ineq}$是用户设置的调整参数。为了实现这一点，我们在优化参数${\bf \xi}$中添加松弛变量$\xi_{ineq}$。这样一来，问题的表述会增加$\lambda_{lin}\xi_{ineq}+\lambda_{quad}\xi_{ineq}^2$，同时添加下面两个不等约束${\bf c}_{ineq} \geq -\xi_{ineq}, \; \xi_{ineq}\geq 0$，其中${\bf c}_{ineq}是如$(15)$中描述的前$n_{ineq}$个约束条件。从物理的角度来看，松弛相当于一个可变大小的支撑多边形，不能小于标称多边形。

### d. _分配一个新的规划：搜索算法。_
下面讨论一下如何将一个新得到的运动规划添加到一个之前已经在执行的运动规划上，避免两者的冲突。为此，我们首先存储求解器优化所需的计算时间$t_c$。我们使用它作为初始猜测来搜索最接近当前测量值的运动规划中的位置，以便过渡到新计划是一个平滑的。为此，我们通过写作来解决线性搜索问题：
$$ 
t= arg\, min_{\bf W}||{\bf p}(t)-{\bf p}_{meas}||_2^2
\tag{18}$$
这里面${\bf W}$是一个正定权重矩阵${\bf p}_{meas}$是完成优化动作后的测量得到的质心位置。


## 4. 落脚点优化：倒立摆模型。
建立如下二次规划问题：
$$\underset{\bf \xi}{min}\quad \frac{1}{2}{\bf \xi}^T {\bf Q}{\bf \xi}+{\bf c}^T{\bf \xi} \quad s. t. \quad {\bf D}{\bf \xi}\leq {\bf f}
\tag{19}$$
这里面${\bf \xi}\in {\Bbb R}^{2n_{feet}}$是裸足点${\bf p}_{f_i}$的$x,y$方向分量，其中$i=1,...,n_{feet}$，$n_{feet}=4$是机器所有脚的总数。与 CoM 运动规划器所做的类似，我们并行优化主控制回路。因此，每当新的优化准备好时，我们都会更新立足点计划。
### a. _成本函数_
相对于默认的战力姿势配置，用户可以自定义一个落脚点位置。这个可以解释为落脚点优化问题的一个正则化项。（正则化的概念需要熟悉一下？）
为了跟踪默认落脚点的位置${\bf p}_{ref_i}$我们为成本函数$(19)$添加下述表述：
$$
{\bf p}_{ref_i} = {\bf W}_{def}, \quad {\bf c}_{def_i} = - {\bf W}^T_{def} {\bf p}_{ref_i}
\tag{20}$$
默认立足点的选择将影响当所有腿与环境接触时足迹的程度。虽然这可能会使小跑步态对干扰更加健壮，但它会在较慢的步行步态中产生更广泛的横向运动。（不知所云？）在实践中，我们在 ANYmal 的实验中可靠地工作的默认立足点位置计算为臀部到地形的垂直投影。

为了跟踪驱动整个运动框架的平均高级速度，我们惩罚与立足点位置的偏差。这些是在优化视界的持续时间内实现恒定速度来计算的。为了避免参考立足点中的跳跃，我们还为当前解决方案和先前计算的解决方案之间的距离设置了成本。
最后，我们为成本函数添加一个稳定项，它是一个倒立摆模型的函数。如参考文献10中所述，这个函数用$\omega ({\bf v}_ref - {\bf v}_{hip_k}\sqrt{h/g}$计算第k个落脚点。这其中，$\omega$是一个正的权重标量，${\bf v}_ref$是高级速度参考，${\bf b}_{hips}$是第k个臀部的速度，$h$是第k个臀部到地面的高度，$g$是加速度常量。（这引导了去关注倒立摆模型的建模？）

### b. _不等约束：不可达落脚点_
为了避免计算违反腿运动学扩展的立足点，我们通过在每个立足点的可行位置上添加不等式约束来利用QP设置。我们通过考虑腿的最大可伸展长度和测量的臀部与地面的高度关系来完成这个约束。
将臀部的位置投影到地形平面上记作${\bf h}_0$，我们设置了描述一个多边形的不等式，该多边形具有均匀分布在${\bf h}_0$附近的$n_p$个顶点。多边形的各个顶点到臀部投影在地形的点${\bf h}_0的距离计算为：
$$\sqrt{l_{max}^2 - h_i^2}$$
（这就是一个直角三角形，斜边长度为腿的最大伸展长度$l_{max}$，高为臀部到地形的投影高度$h_i$，计算落脚点的可行范围，也就是底边长度。）

## 5. 支撑多边形序列生成
>这部分的详细描述要参考文献2。

## 6. 分层次优化计算${\bf \xi}_d$
下面的控制方法采用接触力控制

### a. _运动方程_
$$\begin{bmatrix}
{\bf M}_{fb} & - {\bf J}_{s_{fb}}^T \end{bmatrix} {\bf \xi}_d = - {\bf h}_{fb}
\tag{3}$$
> $XXX_{fb}$: 下标的意思是浮动的主体-'floating base'
* ${\bf \xi}_d$是一个一个$n_u+n_c$行$1$列的向量： ${\bf \xi}_d = \begin{bmatrix} {\bf u}_d^T & {\bf \lambda}_d^T \end{bmatrix} \in {\Bbb R}^{n_u+n_c}$，其中${\bf u}_d^T$是目标节点的加速度，${\bf \lambda}_d^T$是目标接触力；
* ${\bf M}_{fb}$是复合型惯性矩阵的前六行；啥是复合惯性矩阵？
* ${\bf J}_{s_{fb}}^T$是雅克比阵的前六行，它将接触力转换到节点的扭矩；
* ${\bf h}_{fb}$是非线性项的前六行，包括克里奥利力、离心力和重力项；


### b. _接触部分运动约束_
控制器找到的解决方案不应违反 (2) 中定义的接触约束。因此，我们通过设置在接触点施加空加速度:
$$\begin{bmatrix} {\bf J}_s & {\bf 0}_{3n_c \times 3n_c}\end{bmatrix} {\bf \xi}_d = -{\bf \dot J}_s {\bf u} \tag{4}$$
哦~它将${\bf \xi}_d$带进去计算后就得到了：${\bf J}_s  {\bf \xi}_d  = -{\bf \dot J}_s {\bf u}$其实就是公式$(2)$的第二个式子${\bf J}_s  {\bf \xi}_d  + {\bf \dot J}_s {\bf u} = 0$。

### c. _接触力和扭矩限制_

$$
\begin{align}
(_I{\bf h} - {_I{\bf n}}_{\mu})^T {_I{\bf \lambda}}_k\leq & 0 \\
- (_I{\bf h} + {_I{\bf n}}_{\mu})^T {_I{\bf \lambda}}_k\leq & 0 \\
(_I{\bf l} - {_I{\bf n}}_{\mu})^T {_I{\bf \lambda}}_k\leq & 0 \\
- (_I{\bf l} + {_I{\bf n}}_{\mu})^T {_I{\bf \lambda}}_k\leq & 0
\end{align}
\tag{5}$$

$_In$是接触面的法向量；$\mu$是摩擦系数；有了这两个再乘上受力$_I\lambda$，就可以得出最大静摩擦力$_I{\bf n}_{\mu}^T \lambda$。实际在接触点平行于地面方向的两个分力$_I{\bf h}$，$_I{\bf l}$在正反方向上都不应该比这个值大，否则就会发生滑动。这就是公式5约束的由来。

$${\bf \tau}_{min}-{\bf h}_j 
\leq \begin{bmatrix} {\bf M}_j & -{\bf J}^T_{s_j} \end{bmatrix} 
\leq {\bf \tau}_{max}-{\bf h}_j \tag{6}$$
这里的${\bf h}_j$应该就是科里奥利力那一堆东西。
而$\begin{bmatrix} {\bf M}_j & -{\bf J}^T_{s_j} \end{bmatrix}$就是加速度力${\bf M}_j {\bf \dot u}_d^T$和传递力$-{\bf J}^T_{s_j} {\bf \lambda}_d^T$两项的和。它们计算的结果就是电机提供的扭矩力${\bf \tau}$克服完${\bf h}({\bf q}, {\bf u})$剩下的力。

### d. _运动跟随_
为了能跟随浮动主体和摆动腿的目标运动。
我们通过实现具有前馈参考加速度和运动相关状态反馈状态的操作空间控制器来约束关节加速度。
对于主体的线性运动：
$$
\begin{bmatrix}
_C{\bf J}_P & {\bf 0}
\end{bmatrix} {\bf \xi}_d = _C{\bf \ddot r}_{IB}^d 
+ {\bf k}_D^{pos}(_C{\bf \dot r}_{IB}^d-_C{\bf v}_B) 
+ {\bf k}_P^{pos}(_C{\bf r}_{IB}^d-_C{\bf r}_B)
\tag{7}$$
对于主体的角度运动：
$$
\begin{bmatrix}
_C{\bf J}_R & {\bf 0}
\end{bmatrix} {\bf \xi}_d = 
- {\bf k}_D^{ang} {_C{\bf \omega}}_{B}
+ {\bf k}_P^{ang}({\bf q}_{CB}^d \Xi {\bf q}_{CB})
\tag{8}$$

* 雅可比矩阵$_C{\bf J}_P$和$_C{\bf J}_R$是与`控制坐标系C（这是一个与地形局部估计和机器人航向方向对齐的帧）`中表达的基相关的平移和旋转雅可比矩阵。
* ${\bf \Xi}$这个算子产生欧拉向量，表示期望姿态$q^d_{CB}$和估计姿态$q_{CB}$之间的相对方向。
* 这里面${\bf k}_P^{pos}, {\bf k}_D^{pos}, {\bf k}_P^{ang}, {\bf k}_D^{ang}$是用来控制增益的对角正定矩阵。
* 参考的运动$_C{\bf r}_{IB}$和它的导数是运动规划的结果。

### e. _接触力最小化_
可以通过下面的方法将接触力设置为最小值：
$$
\begin{bmatrix}
{\bf 0}_{3n_c \times n_u} & {\Bbb I}_{3n_c \times 3n_c}
\end{bmatrix}
{\bf \xi}_d = 0
\tag{9}$$

### f. _计算电机扭矩_
如果给定了一个优化的节点运动和接触力，${\bf \xi}_d = \begin{bmatrix}
{\bf \dot u}_d^T & {\bf \lambda}_d^T
\end{bmatrix}^T$
我们可以用以下公式计算各个电机的扭矩：
$${\bf \tau}_d = \begin{bmatrix} {\bf M}_j & -{\bf J}^T_{s_j}\end{bmatrix} {\bf \xi}_d + {\bf h}_j$$
其中${\bf M}_j,\, -{\bf J}^T_{s_j}, \, {\bf h}_j$在公式$(6)$已中定义。
> 因此，所有规划的目的就是给出节点的运动加速度${\bf \dot u}_d^T$和接触力${\bf \lambda}_d^T$。
> 