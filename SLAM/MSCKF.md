#####  如何运行程序？

 ```python
 roslaunch msckf_vio msckf_vio_euroc.launch
 rosrun rviz rviz -d rviz/rviz_euroc_config.rviz 
　　　　　
rosbag play ../../../slambook\ 2/V1_02_medium.bag 
 ```

![20200702100826](https://chendaxiashizhu-1259416116.cos.ap-beijing.myqcloud.com/20200702100826.png)

为什么没有建图的部分？

#####  MSCKF的一些理论

- 参考：https://zhuanlan.zhihu.com/p/76341809
- https://zhuanlan.zhihu.com/p/76341809

MSCKF的目标是**解决EKF-SLAM的维数爆炸问题**。传统EKF-SLAM将特征点加入到状态向量中与IMU状态一起估计，当环境很大时，特征点会非常多，状态向量维数会变得非常大。MSCKF不是将特征点加入到状态向量，而是将不同时刻的相机位姿(位置p和姿态四元数q加入到状态向量，特征点会被多个相机看到，从而在多个相机状态（Multi-State）之间形成几何约束（Constraint），进而利用几何约束构建观测模型对EKF进行update。由于相机位姿的个数会远小于特征点的个数，MSCKF状态向量的维度相较EKF-SLAM大大降低，历史的相机状态会不断移除，只维持固定个数的的相机位姿（Sliding Window），从而对MSCKF后端的计算量进行限定。

> EKF-slam什么时候有维度爆炸的问题？估计特征点位置的时候存在维度爆炸的问题？

> 在Iphone上跑MSCKF代码：**Monocular Visual Inertial Odometry on a Mobile Device**

本文给出了用于视觉惯性里程计在线自标定的多状态约束卡尔曼滤波的结果。众所周知，MSCKF在精度和运行时间上都执行传统的EKF SLAM。高精度是两种策略的结果：在EKE中使用特征异常值之前移除特征异常值，在估计特征深度之前等待所有特征测量值可用。MSCKF的运行速度是恒定的，比EKF SLAM快，因为特征没有添加到状态向量中。相反，相机姿态的滑动窗口将保持不变。在我们的实现中，缺点是需要使用IMU的详细模型、具有透镜失真的卷帘相机和相机IMU的空间和时间偏移来进行恒定运动，我们证明这些参数的联合在线校准可以提高里程计的性能。模拟测试表明，所有的校准参数似乎都可以通过足够刺激的运动来观察真实世界的实验表明，苹果iphone 4S和ipad 3视网膜都配备了校准良好的imu，并且只受到较小的尺度偏移。我们推测IMU的温度也有明显的影响。相机的imu空间和速度参数，以及卷帘式快门读取时间和相机畸变参数在测试设备上的多次试验中均收敛到可编程的一致值，MSKCF成功地在长时间内跟踪运动，误差小于行驶距离的1%。在某些情况下，我们证明了该算法对快速运动和旋转具有鲁棒性。将IMU加入到视觉里程计中，不仅对尺度估计有用，而且对高激励运动的鲁棒跟踪也很有用。虽然我们还没有尝试在移动设备上运行该算法，但我们已经证明，它可以在标准笔记本电脑的单核CPU上实时工作。总的来说，我们的结果表明MSCKF是一个精确和稳健的工具，可以使用手持设备进行实时的姿态估计

- 07年开山之作：https://www-users.cs.umn.edu/~stergios/papers/ICRA07-MSCKF.pdf

#####  代码部分（matlab）

参考文献1：The Battle for Filter Supremacy: A Comparative Study of the Multi-State Constraint Kalman Filter and the Sliding Window Filter
参考文献2：Visual Inertial Odometry for Mobile Robotics

********************程序中主要变量说明************************
参数说明：
numLandmarks              每帧特征点个数
camera.c_u                光心横坐标[u pixels]
camera.c_v                光心列坐标 [v pixels]
camera.f_u                焦距 [u pixels]
camera.f_v                焦距 [v pixels]
camera.b                  双目基线距 [m]
camera.q_CI               4x1 IMU到Camera旋转四元数
camera.p_C_I              3x1 IMU坐标系下的Camera位置

msckfParams.minTrackLength        特征点最少相机观测数目
msckfParams.maxTrackLength        特征点最大相机观测数目
msckfParams.maxGNCostNorm         高斯牛顿优化求解三角化特征点的迭代误差上限
msckfParams.minRCOND              矩阵条件数
msckfParams.doNullSpaceTrick      是否做零空间映射
msckfParams.doQRdecomp            是否做QR分解

IMU状态：
imuStates{k}.q_IG         4x1 Global 到 IMU 姿态四元数
imuStates{k}.p_I_G        3x1 IMU 在Global坐标系下位置
imuStates{k}.b_g          3x1 陀螺仪零偏
imuStates{k}.b_v          3x1 速度零偏
imuStates{k}.covar        12x12 IMU 状态协方差

相机状态：
camStates{k}.q_CG        4x1 Global 到 camera 的姿态四元数
camStates{k}.p_C_G       3x1 Global坐标系下的camera位置
camStates{k}.trackedFeatureIds  1xM当前相机可观测到的特征的ID
camStates{k}.state_k     当前相机ID

MSCKF状态：
msckfState.imuState     IMU状态
msckfState.imuCovar     IMU-IMU协方差矩阵块
msckfState.camCovar     camera-camera协方差矩阵块
msckfState.imuCamCovar  IMU-camera协方差
msckfState.camStates    相机状态

特征跟踪列表：
featureTracks        正在追踪的特征点（特征点坐标，能观测到这些特征点的相机，特征点ID）
trackedFeatureIds    正在追踪特征点的ID号         

********************程序步骤***********************************      
步骤1：加载数据
步骤2：初始化MSCKF中imu测量协方差与预测协方差
步骤3：导入测量数据与参考数据
步骤4：初始化MSCKF
      a.将第一个参考值的四元数与位置初始化msckf中IMU的状态
      b.目前MSCKF状态中只有IMU相关的状态，没有camera相关的状态
      c.msckf中不将特征点作为状态，但也会记录跟踪的特征点。初始化时认为第一帧所有特征点都被跟踪上。
步骤5：状态与协方差预测更新。从步骤5到步骤10循环进行。
      a.状态预测更新。
           a1.IMU状态更新（角速度更新四元数，速度积分更新位置，陀螺仪零偏和速度零偏不变）
           a2.camera状态更新（保持和上一次相同）
      b.协方差预测更新
           b1.IMU-IMU状态的协方差更新，并对角化。（imuCover := P * imuCover * P' + G * Q_imu * G'）
           b2.IMU-camera状态的协方差更新。（imuCamCovar := P * imuCamCovar）
           b3.camera-camera状态的协方差更新。（camCovar:=camCovar）
步骤6：状态增广，在msckf状态中增广相机状态
      a.由IMU以及IMU与camera的固连关系得到相机的位置和姿态
      b.增广雅克比。增广状态以后，需要得到增广状态（相机位置、相机四元数）对msckf状态（增广前以后的状态）的雅克比
      c.增广预测协方差矩阵，更新增广后的协方差矩阵
步骤7：遍历当前帧所有特征点，更新featureTracks
      说明：msckf中用featureTracks记录了目前被跟踪到的特征点。
           featureTracks中包含每个特征点ID和观测值（即在所有能观测到该特征点的相机坐标系下的齐次坐标）
      a.遍历当前帧所有特征点，判断是否属于featureTracks
      b.如果该特征点在视野范围内：将特征点在相机坐标系下的齐次坐标添加到featureTracks中对应特征的观测中。
      c.如果该特征点超出视野范围或者能够观测到该特征的相机数目大于上限值
            c1.从所有相机状态中剔除该特征点，并将涉及到的相机状态添加到状态待优化列表中
            c2.如果待优化的相机状态超过最小跟踪长度（10），则将其添加到列表中用于优化
            c3.若已使用完给特征，则从featureTracks中剔除该特征点
      d.如果该帧检测的特征点视野范围内（但是不属于featureTracks），则将其添加至featureTracks中
步骤8：MSCKF测量更新。遍历所有用于优化的特征点,构造观测模型（特征点重投影误差），更新MSCKF状态
      a.通过特征点所有观测估计出该特征点的3D空间坐标位置
      b.通过特征3D坐标与相机匹配特征点之间的重投影残差构造观测模型，包括重投影误差对MSCKF状态量的雅克比矩阵的求解 
      c.计算卡尔曼增益，更新误差状态
      d.根据误差状态更新MSCKF状态，x_true := x_nominal + detx
      e.更新MSCKF测量协方差矩阵
步骤9：历史状态更新。从MSCKF状态中更新IMU的历史状态，通过相机的状态更新对应时刻imu的位姿状态
       说明：重投影误差只和相机状态有关，msckf测量更新只能更新相机有关的状态，因此在步骤8测量更新后需要通过相机状态更新IMU状态
步骤10：剔除MSCKF中需要被删除的状态和对应的协方差矩阵块
       如果相机不能观测到featureTracks中的任一特征（对重投影误差无贡献），则MSCKF状态中剔除该相机状态

####  论文中的结论

#####   2007年讨论

我们现在研究所描述算法的一些性质。如前一节所示，滤波器的计算复杂度与所服务的特征数成线性关系。状态向量中包含的状态数最多为立方。因此，包含在状态中的姿势的数量是决定算法计算成本的最重要因素，因为该数量是一个可选择的参数。它可以根据可用的计算资源和给定应用程序的精度要求进行调整。如果需要，过滤器状态的长度也可以在过滤器运行期间自适应控制，以适应资源的不同可用性。

<span style="color:blue;">**利用相机观测值进行递推状态估计的一个困难来源是测量模型的非线性性质**</span>。基于视觉的运动估计对噪声非常敏感，特别是当观测到的特征距离较大时，虚假的局部极小值会导致收敛到不一致解[123]。文献中利用Sigma点Kalman滤波[24]、粒子滤波[4]和特征25的反深度表示等技术解决了非线性引入的问题。所描述的al gorithm的两个特性增强了其对线性化误差的鲁棒性：（i）测量模型中使用的反向特征深度参数化（参见附录）和（ii）测量的延迟线性化[17]。通过该算法的构造，在对特征进行EKF更新之前，对每个特征的多个观测值进行收集，从而对测量雅可比进行更精确的评估.

一个有趣的观察是，在典型的图像序列中，大多数特征只能在少数帧上可靠地跟踪（机会性特征），并且只有ew可以被长时间跟踪，或者在重新访问位置时（持久性特征）。这是由于摄像机的视场有限以及遮挡图像噪声和视点的变化，导致了特征跟踪算法的失败。如前所述，如果已看到特征的所有姿态都包含在状态向量中，则所提出的测量模型是最优的，除了线性化不精确外。因此，对于真实图像序列，所提出的算法能够最优地利用机会性特征的定位信息，并且我们注意到，状态向量Xk不需要只包含IMU和相机姿态。如果需要，持久性特征可以包含在过滤状态中，并用于SLAM。这将进一步提高在长循环区域内可达到的定位精度。




$$
{ }^{G} \boldsymbol{p}_{G p s}={ }^{G} \boldsymbol{p}_{I}+{ }_{I}^{G} \boldsymbol{R} \cdot{ }^{I} \boldsymbol{p}_{G p s}
$$