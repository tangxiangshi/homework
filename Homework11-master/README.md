![image](https://github.com/Robotics-Aerial-Robots/Homework6/blob/master/Figures/LOGO%20%E4%B8%AD%E8%8B%B1%E6%96%87%E6%A9%AB.png)
# 108 年度 機器人學：多軸旋翼機 

### Homework11
Deadline: June 2

### Topic
---
Please complete the following two parts for this homework.

1. Please finish the update function in kalman.cpp (you can check out page 20 of the ppt. from week_12)
2. Please set parameters in the kf_test.cpp by using the following dynamic model.

Dynamic model:

<img src= "https://github.com/Robotics-Aerial-Robots/Homework11/blob/master/photo/1.png" width="40%" height="20%">	
	
<img src= "https://github.com/Robotics-Aerial-Robots/Homework11/blob/master/photo/11.png" width="90%" height="20%">


Suppose that the car is equipped with a position sensor that measures its output y with an additive noise v, please find the state estimate (i.e., position, velocity, acceleration) and compare it with the true state.

### Real data
The initial condition of the true state is x0=[0; 0; 0].

### Instruction
```
  roscore
  rosrun hw11 kf_test
```
