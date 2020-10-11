#### 简介

这里是一个用数值模拟的大摆角单摆。

众所周知的是单摆的是近似的简谐运动当且仅当: 
$$
-5°\leq\theta\leq5°
$$
由于简谐运动需要满足：
$$
F_x\approx kx
$$
而在摆角小于5°的时候我们有：
$$
F_x=mg*tanx\approx mg*sinx
$$
而引入一个大摆角时，传统的解析法变得不再使用，我们使用数值模拟。

同时数值模拟可以很好的计算具有阻尼的特殊情况，比如引入一些流体力学的受力参数对加速度进行修正。

本程序使用c++编写。<del>当然是因为跑的比较快所以适合做大量的浮点数运算</del>

将每一秒拆分为1e6个时间单元进行模拟，每1e5个时间单元产生一个采样点输出该单摆的当前位置。

<del>由于不会gui所以暂时只能在rqt_plot里查看单摆的运行情况。</del>

#### 使用说明

参照程序内注释，单位均为国际基本单位。

初始摆长length=10，重力 vec_G=9.8。

摆的释放位置为（10,0），在程序中由x.position给出，当前速度方向由x.velocity_direct给出。

摆球的速度为x.velocity，在圆周上的弧度为x.radian， 

c1,c2为阻尼修正参数，c1为x.velocity^2项 的系数，在稀薄流体中适用，c2为x.velocity项的系数，在粘稠流体中适用。

给定的初始值可以自行在程序内修改。

使用时在ROS工作空间下使用命令：

```
catkin_make --force-cmake
```

强制编译，然后运行

```
rosrun pend pend
```

单摆的的x，y坐标将会分别发布在pend_x, pend_y上

用rqt_plot查看效果如图：

![233.png](https://s1.ax1x.com/2020/10/10/0spBM4.png)



