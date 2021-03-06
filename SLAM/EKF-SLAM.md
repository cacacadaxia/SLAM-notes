### EKF-SLAM

[TOC]

#### 文摘

本文分析了EKF-SLAM的扩展卡尔曼滤波公式。我们证明车辆航向角的“真实”的不确定性超过一个限度，该算法会产生非常较好的结果。这种故障是微妙的，一般来说，如果没有基本的事实，就无法检测到，尽管非常不一致的滤波器可能会显示出可观察的症状，例如车辆姿态更新中的过大跳跃。添加稳定噪声、使用迭代EKF或无迹滤波器等常规解决方案并不能改善这种情况。但是，如果保持smal1航向不确定性。EKF-SLAM在较长时间内表现出一致的行为。尽管不确定性估计慢慢变得乐观，但通过应用诸如批量更新或稳定噪声等策略，可以无限期地减少不一致性。小航向方差SLAM的可管理退化表明了submap方法对大比例尺地图的有效性。

#### 引言

史密斯等人对SLAM问题的原始随机解。[15] 现在已经有将近20年的历史了，而且这个概念已经达到了一个成熟的状态，足以允许在具有挑战性的环境中实际实现已经解决的重要问题是在混乱的环境中可靠的数据关联1,14]；通过局部分区数据融合16]、6]和保守数据融合6]、112、111以及与子映射I1]31的边界累积非线性来降低计算复杂度，然而，尽管SLAM算法在实际应用中取得了明显的成功，其基本一致性却很少受到关注。

非线性SLAM主要实现为扩展卡尔曼滤波器（EKF），其中系统噪声假设为高斯，非线性模型线性化以适应卡尔曼滤波算法。EKF-SLAM用近似均值和方差表示状态不确定性。这是两方面的问题。首先，由于线性化，这些力矩是近似imate的，可能与真实的“第一和第二力矩”不精确匹配。第二，真实概率分布是非高斯的。因此，即使是真实的均值和方差也可能不是一个足够的描述。这些因素影响SLAM概率分布在运动和测量序列上的时间投影方式，以及近似误差的累积方式。


