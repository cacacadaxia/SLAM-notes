#####  Indirect Kalman Filter for 3D Attitude Estimation

卡尔曼滤波器由两个阶段组成，用于确定当前时刻的估计值和一些本体感测姿态。在传播的第一阶段，滤波器根据最后的估计值对姿态进行预测。然后在更新阶段对该估计进行修正，其中考虑了新的绝对定向测量，因此预测系统状态的方法是将给系统的控制命令输入到系统模型中，从而预测系统的行为。估计位置和方向的另一种方法是使用来自惯性测量单元（MU）的数据作为动态模型替换。IMU提供对作用在系统上的平动加速度和旋转速度的测量。在本文中，我们将在讨论状态方程和误差传播之前采用后一种方法，我们将描述提供转速测量的陀螺模型。

> 后一种？

作为惯性测量单元的一部分，三轴陀螺提供转速测量。众所周知，陀螺仪受到不同的误差项的影响，例如速率噪声误差和偏差。根据文献[3，6]。我们使用一个简单的模型，将测量的转率wm与实际角速度w联系起来

![image-20200623074243928](/Users/chenshiming/Library/Application%20Support/typora-user-images/image-20200623074243928.png)

在这个方程中，b表示陀螺偏置，nr表示速率噪声，假设为具有特征的高斯白噪声

![image-20200623074250727](/Users/chenshiming/Library/Application%20Support/typora-user-images/image-20200623074250727.png)





在离散情况下，nrd和nwd必须具有与转动率相同的单位（参见第2.2节中的状态方程），即Iad。为了保持噪声强度的等价性，我们必须在离散化时加入采样频率。

Simon 17进一步解释了连续和离散噪声方差之间的转换。第230-233页]。为了与书中的推导进行类比，偏压驱动噪声nww对应于过程噪声，而速率噪声n对应于测量噪声。关于该陀螺噪声模型及其与人工提供的各种规范之间关系的详细说明，见[8]的附录。

![image-20200623081417100](/Users/chenshiming/Library/Application%20Support/typora-user-images/image-20200623081417100.png)



![image-20200623081235387](/Users/chenshiming/Library/Application%20Support/typora-user-images/image-20200623081235387.png)

![image-20200623081317016](/Users/chenshiming/Library/Application%20Support/typora-user-images/image-20200623081317016.png)