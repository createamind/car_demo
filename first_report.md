### 1.概述
作为泊车场景下自动驾驶算法研究的基础调研，主要分析仿真器的能否满足功能上的需求，以及性能上的约束，demo的实现。

仿真器作为自动驾驶算法的试验工具，应该满足：

a.重现泊车场景，提供不同难度的场景供后续迭代；
```
有可调整的参数设置不同场景,设置车位位置数量，车辆位置，障碍物
```
b.与控制算法交互：输入为控制算法发出的指令，并按指令控制车辆转向加油刹车；以及对环境的控制信号，将环境初始化到某一状态。
```
control command： steering ，throttle，brake ，shift_gears
env command ： reset ，set
```

输出为车辆处在的环境的状态信息，间接传感器数据或者直接的状态
```
sensor ： image ， pointcloud ，laserscan ，contact, sonar
直接的状态 ：车辆的位置方向 modelstate
```

c.以及根据位置与目标差异，碰撞信息等计算reward，判断在某种条件下终止回合并重启。
```
reward=f(state)

```



### 2.传感器及数据
配置的传感器有 4 x camera，2 x laser，1 x lidar,8 x sonar

camera 生成的数据为 320 x 240 的图像，发送频率 30 HZ；
占用带宽4 x 7MB/s msg类型为 sensor_msgs.Image 经过 cv2_bridge 可转换成 numpy 数组使用。  
format: numpy.ndarray(shape=(240, 320, 3), dtype=uint8) [front, left, right, back]

laser 生成的数据为 0.2m到3m(?)距离范围内，水平方向上 点的距离信息，发送频率 30 HZ； msg类型为 sensor_msgs.LaserScan， 经过 laser_geometry 包 可转换成 pointcloud 使用 ，或者转换成矩阵。  
format: ?

lidar 生成的数据为0.2m到3m距离范围内，8192个点的坐标，发送频率 30 HZ， 占用带宽 2MB/s； msg类型为 sensor_msgs.PointCloud， 可转换成矩阵使用。  
format: ?

sonar 生成0.2m到5m，直线方向上的障碍物距离，发送频率 5 HZ； msg类型为 sensor_msgs.Rang， 距离为float ，直接使用。  
format：？

碰撞传感器，生成碰撞点坐标信息，撞击力，可转换成相对于车辆的位置，估算伤害值，作为reward。

位置：  
format：？

数据传输的性能压力并不大，主要是硬件性能的约束。

### 3. 控制信息
根据泊车的低速环境，设定的控制信号输出频率为5hz
steering：  
format：？range：？

accel:  
format: ? range: ?

brake:  
format: ? range ?

gear:   
format: ? range ?

### 4.问题
- 需要注意的是各传感器初始参数的扫描范围较小，死角较大，可以通过修改laser ，sonar扫描夹角参数获得更大的感知范围。或者考虑融合不同传感器的优势来避免死角。

- 由于路牙比较低，雷达无法感知，目前会通过视觉来感知，建议使用对比度比较强烈的颜色来构造路牙（顶部／侧面／路面颜色分开）
- 实际仿真过程中laser，和lidar 数据生成频率 5HZ左右，后续可采用gpu加速的插件。

- 实际仿真过程运行比较慢，本机cpu占用140%，配置的参数要求仿真器内与外部时间比例为1，实际0.6左右，后续多实例并行需要更强主机。(评估出达到1:1的cpu的最低配置需求)

- 碰撞传感器坐标换算不能自动换算，采用手动计算的方法，可能有问题，后续观察。

- 内部数据生成是异步还是同步模式（如4个摄像头的都是30帧，如果是异步模式，会导致队列数据不同步，处理上的逻辑需要复杂一点）

### 5.结论
- 功能
  * 需求
  * 迭代
  
- 性能
  * 目前来看，单实例即使在未去除GUI的情况下也满足训练需求
  
- 风险
  * 多实例下未测试
  * 如目前视频采集的分辨率(320x240)不能满足泊车训练需求，那性能问题会比较突出，可以采购高主频的CPU来临时解决
  * 
