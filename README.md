# yolo角度对准开发记录
```
开发人员：杨工

工作记录：
————2023年4月22号
1、麦克纳姆轮小车模型建立，添加雷达、摄像头、陀螺仪数据
2、移殖yolo程序，ROS封装

————2023年4月26号
1、cv_bridge调试

————2023年4月27号
2、逻辑调整根据输出的count发布停止速度
```

**系统环境**

```
ubuntu 18.04
ros melodic
```

**启动步骤**

1、启动yolo检测：
```
roscore
rosrun opencv_test yolo
```

输出ros的话题/stop_flag
```
---
data: True
---
data: True
---
data: True
---
data: False
---
data: False
---
data: False
---
```
为true的时候，小车暂停；为false，小车旋转

2、启动旋转控制器
```
rosrun opencv_test rotation
```
旋转控制器输出角速度，小车进行对准

**检测效果**

![Image text](https://github.com/haicheng12/yolo_rotation/blob/main/img/yolo.png)

**优化项**

1、实车调试







