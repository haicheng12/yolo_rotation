# yolo角度对准开发记录
```
开发人员：杨工

工作记录：
————2023年4月22号
1、麦克纳姆轮小车模型建立，添加雷达、摄像头、陀螺仪数据
2、移殖yolo程序，ROS封装

```

**系统环境**

```
ubuntu 18.04
ros melodic
opencv 4.5.5
安装编译opencv参考：https://blog.csdn.net/dfman1978/article/details/124085664
用到的模型和视频（注意修改加载的路径）：https://github.com/haicheng12/yolo_model
```

**启动步骤**

1、启动yolo检测：
```
roscore
rosrun opencv_test yolo
```

输出ros的话题/angle
```
rostopic echo /angle
data: 31.9301363932
---
data: 12.4779459564
---
data: 31.8026922791
---
data: 12.4779459564
```

2、启动旋转控制器
```
rosrun opencv_test rotation
```
旋转控制器输出角速度，小车进行对准

3、启动小车仿真
```
roslaunch atom atom_world.launch
```

**检测效果**

![Image text](https://github.com/haicheng12/yolo_rotation/blob/main/img/yolo.png)

**优化项**

1、需要将gazebo仿真的相机的输入转化给yolo使用
2、需要识别gazebo的柱子
3、实车调试







