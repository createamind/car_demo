1. build docker 

```
./build_demo.bash
```

2. run_demo
```
./run_develop.bash
```

3. launch gazebo
```
roslaunch car_demo demo.launch
```
4. test rl demo
```
python train/trian_ddpg.py
```
trian_ddpg.py
采用ddpg算法，输入状态为图像，输出动作为连续的转角油门刹车。
单实例，记忆回放。
-  todo: 
0.设计reward function
1.memory replay 优化 ，如何保存大量的图像数据
2.感知，先识别检测图像中的障碍物等（需要带标签的数据）
