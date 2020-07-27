## MEKF

[TOC]

### 1. 随机过程

[直观上理解](https://zhuanlan.zhihu.com/p/71202672)，**因为近似的高斯白噪声在每一时刻的方差都相同，所以其对于时间积分的方差会随时间线性增长**。如果把方差理解为信号的不确定性，这实际上是不确定性在随着时间线性积累：每过一点时间，积分增量都不确定，因此随着时间增长，积分信号越来越不确定。可以设想，如果 IMU 的角速度测量噪声是高斯白噪声，那么它积分得到的角度会越来越不准确，这是 IMU 角度漂移的原因。

<img src="https://chendaxiashizhu-1259416116.cos.ap-beijing.myqcloud.com/20200526145610.png" alt="20200526145610" style="zoom:50%;" />

> 上面应该是“Yt的积分的功率谱密度是。。”。上图可知，角速度的功率谱密度是由陀螺仪来决定的。那么怎么从功率谱函数去确定Q？

最后，如果把维纳过程进一步关于时间积分（integrated Wiener process），可以想象得到的信号的不确定性随时间增长速度会更快。事实上，我们有 <img src="https://www.zhihu.com/equation?tex=%5Cmathrm%7Bvar%7D%5Cleft%28%5Cint_0%5EtW_sds%5Cright%29%3D%5Cfrac%7B1%7D%7B3%7Dt%5E3" alt="[公式]" style="zoom: 110%;" /> 。在惯性导航中，如果加速度计的测量噪声是高斯白噪声，那么积分得到的位置是维纳过程关于时间的积分，不确定性会随时间的三次方增长。如果考虑到需要用陀螺仪积分的角度把加速度变换到地球坐标系中，角速度的误差反应到位移中是维纳过程的积分的积分，其不确定性的增长速度会更快，<span style="color:blue;">**这也是为什么我们使用的便宜的 IMU 只适用于估计角度，而无法积分得到位移**</span>。

所以可以看到加速度两次积分的随机过程更加复杂，属于二阶系统。最后的结果就是廉价的加速度计的数值基本不能直接积分单独使用。

#####   那么到底什么是功率谱密度函数？

[参考](https://baike.baidu.com/item/%E5%8A%9F%E7%8E%87%E8%B0%B1/6463147)，功率谱表示如下：
$$
P(\omega)=\lim _{T \rightarrow \infty} \frac{\left|F_{T}(\omega)\right|^{2}}{2 \pi T}
$$

怎么从代码中获取定义呢？

有下面的关系：
$$
S(\omega)=\int_{-\infty}^{\infty} \tilde{K}(\tau) e^{-i \omega \tau} d \tau
$$
也就是说功率谱密度和协方差函数互为傅里叶变换的关系。在一个积分系统中，x作为输入，y作为输出，通过这个性质，可以不严格地得到$S_{Y}(\omega)=|H(\omega)|^{2} S_{X}(\omega)$ ，那么关于时间的积分的功率谱密度是 $\frac{1}{\omega^{2}} S_{x}(\omega)$。

### 2. 乘法扩展卡尔曼滤波器（MEKF）

#### 1 理论部分

> -  [主要参考](https://www.zhihu.com/people/wang-wei-xin-72-20/posts)
> - [主要参考](https://zhuanlan.zhihu.com/p/84010704)

使用欧拉角设计了能够实现 IMU 姿态估计的卡尔曼滤波器。<span style="color:blue;">并且我们发现在俯仰角等于pi/2时，因为欧拉角的万向节锁死现象，滤波器的协方差矩阵的递推公式变得很不准确</span>。在这篇文章中，我们介绍乘法扩展卡尔曼滤波器（Multiplicative Extended Kalman Filter, MEKF），来解决这一问题。

在国内论坛讨论 IMU姿态估计的大量文章中，算法集中在对卡尔曼滤波器的直接应用，及Robert Mahony等人提出的互补滤波器。然而目前在导航领域最通用的算法并不是这两种，而是1982年提出的一种名为“Multiplicative Extended Kalman Filter”，直译是“乘法扩展卡尔曼滤波器”的方法。

旋转矢量和欧拉角都具有奇异性，也就是说在某种姿态下，它们的导数会趋向无穷大；四元数没有奇异性，但它和姿态是2比1的对应关系；只有旋转矩阵既没有奇异性，而且和姿态是一一对应的。

<span style="color:blue;">**在介绍互补滤波器的时候我们提到，互补滤波器是将两个信号通过 G(s)与1-G(s)两个滤波器，再进行相加，这实际上也是加权平均的过程。对比“互补滤波器”那篇文章中的公式 (6) 和这篇文章中的公式 (6) 可以看到，两者的区别在于卡尔曼滤波器的权重是随时间变化的，而互补滤波器不变。从公式 (2) 和 (4) 可以看到，每进行一次迭代，**</span>预测值的准确性是随噪声的积累和测量的到来不断变化的，而卡尔曼滤波器会自动调整权重使估计的结果最优。在很多关于 IMU 姿态估计的文献中，作者将不使用欧拉角的原因归结于欧拉角在俯仰角计算时会有“万向节锁死”的特性，这种说法是正确的。

综上所述，<span style="color:blue;">**MEKF 经过 50 年的发展，不仅具有完善的理论，而且经过了最为严格的实验验证，我认为是所有使用 IMU 进行姿态估计算法中的不二之选。但 MEKF 在民用领域推广的过程中名字发生了一些变化，例如在机器人领域，它被称为 error state Kalman filter [5]，和 indirect Kalman filter [6]**</span>。这些变化在我们查阅资料时会带来不少额外的困难，可能也是它没有 Mahony 的互补滤波器知名度高的原因。我写这个专栏的主要目的就是介绍 MEKF ，希望这个在航天领域的标准算法能被更多使用 IMU 的朋友们用在自己的工作科研中。

#####  对于理论部分的理解

<span style="color:green;">**MEKF本质上来说也就是ESKF，博主这里的方法与ESKF的区别就是没有去考虑到陀螺仪的角度积分带来的随机游走部分。**</span>

<span style="color:green;">**在姿态估计的问题中，比较重要的一个概念就是三维光滑流行空间的正切空间是一个欧式空间的问题。广泛的应用到姿态估计的各个方面中**</span>。

#####  试验部分

> MEKF: 误差状态量扩展卡尔曼滤波，摘取博客中的做法，记录如下：
>

![20200518164858](https://chendaxiashizhu-1259416116.cos.ap-beijing.myqcloud.com/20200518164858.png)

程序中的做法如下：

![20200518165212](https://chendaxiashizhu-1259416116.cos.ap-beijing.myqcloud.com/20200518165212.png)

现在有代码如下，基于博主提供的开源项目进行修改。

> 需要提前导入数据，需要测量四元数。
>
> <span style="color:blue;">**那么一个问题：测量四元数怎么获得呢？如果利用地磁计的信息，不会带来角度问题吗？貌似是会的，那么怎么办？**</span>（主要是对于协方差的影响）

 ```matlab
% % 前提是有角度测量的方式，那么就比较有效
clear all;
close all;
N = 10;
sf = 200;

[qTrue,gyro] = genTrig(N,sf);
qMea = genMea(qTrue);
q0 = mulQua1(qTrue(1,:),expQua([pi,0,0]));

%%
dt = 1/sf;
Nt = length(gyro);

% rotation or vector measurement
if isempty(qMea)
    meaType = 'V';
else
    meaType = 'A';
end

% default parameters
U.P0 = 100;
U.GyroAngleRW = 0.1*pi/180;
U.rvstd = 0.1;
U.vMeaStd = [0.1,0.1];

% U = parseVar(varargin,U);

% Q and R
Q = eye(3)*U.GyroAngleRW^2*dt;
R = eye(3)*U.rvstd^2;%%重要


% pre-allocate memory
qEst = zeros(Nt,4);
P = zeros(3,3,Nt);
if strcmp(meaType,'V')
    qMea = zeros(Nt,4);
end

% initialize
qEst(1,:) = q0;
P(:,:,1) = eye(3)*U.P0^2;

% filter iteration
for nt = 2:Nt
    % integration
    av = 0.5*(gyro(nt-1,:) + gyro(nt,:));%%求中位数
    qEst(nt,:) = mulQua1(qEst(nt-1,:),expQua(av*dt));%%为何不用旋转矩阵的？
    
    % uncertainty propagation
%     第一种
    F1 = expRot(av*dt)';
%     第二种
    S = omegaMatrix(av*dt);
    normV  = sqrt(S(1,2)^2+S(1,3)^2+S(1,3)^2);
    F2 = eye(3)+sin(normV)/normV*S(:,:)+...
            (1-cos(normV))/normV^2*S(:,:)^2;
%     第三种    
%%和罗德里公式一样？
    rotation_part = av*dt;
    rotation_part = rotation_part';
    %%李代数指数映射的大小
    theta = sqrt(rotation_part'*rotation_part);         % 求的是向量的模
    %%实现的时候先计算出1/theta，然后使用乘法替换下面所有的除法。
    a = rotation_part/theta;            % 这个是什么？
    c = cos(theta);
    s = sin(theta);
    F3 = c*eye(3)+(1-c)*a*a'+s*omegaMatrix(a);
    
    F1-F2
    F3-F2
%     F1 - F3
    %%
    F = F1;
    P(:,:,nt) = F*P(:,:,nt-1)*F'+Q;
    % update
    K = P(:,:,nt)*(P(:,:,nt)+R)^-1;     %%K始终是3维的，这个和协方差的性质相关的
    dv = K * logQua1(mulQua1(invQua1(qEst(nt,:)),qMea(nt,:)))';
    qEst(nt,:) = mulQua1(qEst(nt,:),expQua(dv)');
    P(:,:,nt) = (eye(3)-K)*P(:,:,nt);
end

%% 代码某一部分是存在问题的，暂且不管
l1 = sum(qEst - qTrue,2);
l2 = sum(qEst + qTrue,2);
if abs(sum(l1(end-5:end)))<0.1
   qEstMerr =  qEst - qTrue;
elseif abs(sum(l2(end-5:end)))<0.1
   qEstMerr =  qEst + qTrue;
else
    fprintf('fuck , whats wrong\n');
end

figure;
subplot(2,1,1)
plot(qEstMerr);
subplot(2,1,2)
plot(qMea - qTrue);

%% my own (肯定就飘了)
function [ q ] = mulQua1( q1, q2)
% 计算两个单位四元数的乘法
q1 = q1';
q2 = q2';
% multiplicate
q(1,:) = q1(1,:).*q2(1,:)-q1(2,:).*q2(2,:)-q1(3,:).*q2(3,:)-q1(4,:).*q2(4,:);
q(2,:) = q1(1,:).*q2(2,:)+q1(2,:).*q2(1,:)+q1(3,:).*q2(4,:)-q1(4,:).*q2(3,:);
q(3,:) = q1(1,:).*q2(3,:)-q1(2,:).*q2(4,:)+q1(3,:).*q2(1,:)+q1(4,:).*q2(2,:);
q(4,:) = q1(1,:).*q2(4,:)+q1(2,:).*q2(3,:)-q1(3,:).*q2(2,:)+q1(4,:).*q2(1,:);
q = q';
end

function [ v, u, theta ] = logQua1( q )
% 四元数的对数映射
% check size and unitness
q = q';
% calculate log
normQv = sqrt(sum(q(2:4,:).^2));
u = q(2:4,:)./normQv;
theta = wrapToPi(atan2(normQv,q(1,:))*2);

% identity
indi = find(q(1,:)==1);
if ~isempty(indi)
    u(:,indi) = [1;0;0];
    theta(indi) = 0;
end
% format result
v = u.*theta;
v = v';
end

function [ invq ] = invQua1( q, checku )
q = q';
invq = [q(1,:);-q(2:4,:)];
invq = invq';
end



function [ q ] = expQua( v, check0 )
% calculate the exponential map from pure quaternion to unit quaternion
% If v is a 3-by-n or n-by-3 matrix, treat v as n 3-vectors
% If v is a 4-by-n or n-by-4 matrix, treat v as n pure quaternions
% in this case, if check0==true (default), check if v is pure quaternions
% q returns n unit quaternions in the same dimension as v

if ~exist('check0','var') || isempty(check0)
    check0 = true;
end

% check dimensions and pure quaternion
if size(v,1)==3
    tran = false;
    v = v/2;
elseif size(v,2)==3
    v = v'/2;
    tran = true;
elseif size(v,1)==4
    if check0
        if ~isempty(find(v(1,:)~=0,1))
            error('v must be pure quaternions');
        end
    end
    tran = false;
    v = v(2:4,:);
elseif size(v,2)==4
    if check0
        if ~isempty(find(v(:,1)~=0,1))
            error('v must be pure quaternions');
        end
    end
    tran = true;
    v = v(:,2:4)';
else
    error('v must be of size 4-n, n-4 for pure quaternions or 3-n, n-3 for vectors')
end

% calculate exponential map
theta = sqrt(sum(v.^2));
q = [cos(theta);v./theta.*sin(theta)];

if ~isempty(find(theta==0,1))
    q(:,theta==0) = [1;0;0;0];
end

% format result
if tran
    q = q';
end

end

function [omega]=omegaMatrix(data)
% wx=data(1)*pi/180;
% wy=data(2)*pi/180;
% wz=data(3)*pi/180;
wx=data(1);
wy=data(2);
wz=data(3);
omega=[
    0,-wz,wy;
    wz,0,-wx;
    -wy,wx,0
    ];

end
 ```

#### 2 备用内容

**其中使用欧拉角的积分方法是其运动学方程的近似解，而使用旋转矩阵和单位四元数的积分方法是运动学方程的解析解。因此我更推荐使用旋转矩阵或四元数对角速度进行积分。**

bias 和白噪声不太一样，白噪声不能消除，只能在积分之后消除；bias 差不多可以理解为另一个白噪声的积分。

> 对于MEKF的笔记整理如下。下面介绍了两种方法，一种是对四元数进行处理和估计，另外一种是利用旋转矩阵来估计的。但均是利用差分量去处理的。
>
> 另外一个问题就是F为什么是这个结果让人很困惑。

![20200526142444](https://chendaxiashizhu-1259416116.cos.ap-beijing.myqcloud.com/20200526142444.png)

<span style='color:red'>上面利用了旋转表示中的李代数的内容。即所谓李代数与李群之间的指数映射与对数映射之间的关系</span>。

可以看到，这里主要用了几个假设：

注意这里我们并没有更新 $\delta \boldsymbol{\theta}$的值，这是因为卡尔曼滤波器是无偏估计，所以$\delta \boldsymbol{\theta}_{n-1 | n-1}=0$。而且我们假设陀螺仪噪声的均值为零，所以$\delta \boldsymbol{\theta}_{n | n-1}=0$ ，它只表示状态变量R的误差。关于 QIMU 的理论取值可以参考上一篇文章中对欧拉角卡尔曼滤波器的讨论。

#### 3 利用小觅相机的角度估计

利用ESKF的方法。

