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
-  todo: 
1.memory replay 优化 ，如何保存大量的图像数据
2.如何感知，先识别检测图像中的障碍物等（需要带标签的数据）
