### ROS机器人

[TOC]

#### 0 ROS相关工具的使用

#####  相关资料

> - ros[通信架构](https://zhuanlan.zhihu.com/p/144602033)，[这个专栏写的很棒](https://zhuanlan.zhihu.com/c_1243036632745091072)；

##### 一个简单的实例

- rosbag负责对数据的录制与播放
- 自带的话题有如下，没有程序运行的时候也有。
  - rosout
  - rosout_agg
  - clock
- 录制：rosbag record [topic1]，同时会产生topic进行
- 程序自带的topic：
  - aruco_slam/pose
  - aruco_slam/odmtery(里程计)
- 播放：rosbag play -r4 [msg_bag]
  - 在rostopic list时会多出几个topic

#### 1 ROS通信机制

##### 1.1 通信

<img src="https://chendaxiashizhu-1259416116.cos.ap-beijing.myqcloud.com/ROS.jpeg" alt="ROS" style="zoom:200%;" />

首先提出ros的整体架构：

![20200520075544](https://chendaxiashizhu-1259416116.cos.ap-beijing.myqcloud.com/20200520075544.png)

举一个实例如下：

![20200520075227](https://chendaxiashizhu-1259416116.cos.ap-beijing.myqcloud.com/20200520075227.png)

sever与client之间的关系如下：

![20200520075444](https://chendaxiashizhu-1259416116.cos.ap-beijing.myqcloud.com/20200520075444.png)

代码部分[参考](https://zhuanlan.zhihu.com/p/64562169)博客总结如下：

![20200520081114](https://chendaxiashizhu-1259416116.cos.ap-beijing.myqcloud.com/20200520081114.png)

##### 1.2 launch文件解析

```python
<launch>

  <node pkg="occ_grid_mapping" type="odometry" name="odometry" output="screen" clear_params="true">
						<rosparam file="$(find occ_grid_mapping)/config/default.yaml" command="load" />
  </node>

  <node pkg="occ_grid_mapping" type="mapping" name="mapping" output="screen" clear_params="true">
						<rosparam file="$(find occ_grid_mapping)/config/default.yaml" command="load" />
  </node>
  
	<node pkg="rviz" type="rviz" name="rviz" output="screen" 
      args="-d $(find occ_grid_mapping)/rviz/default.rviz" required="true">
    </node>

</launch>
```

这里有两个节点，occ_grid_mapping是功能包的名字，odometry是节点的名字。另外调用rviz软件并且显示数据。

##### bashrc

 	

#### 2 机器人制作

[参考博客](https://zhuanlan.zhihu.com/p/68186600)。

下面介绍了STM32电机驱动板与上位机（树莓派或者arm）之间的关系示意。

![20200520082657](https://chendaxiashizhu-1259416116.cos.ap-beijing.myqcloud.com/20200520082657.png)

![20200520082615](https://chendaxiashizhu-1259416116.cos.ap-beijing.myqcloud.com/20200520082615.png)



#### 3 机器人的通信问题	

- [多机器通信](https://zhuanlan.zhihu.com/p/101331694)；ROS通信机制；
- 