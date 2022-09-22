# 前言
记录无人车开发中的代码，以备后续查找

# dev_ws-V1.0
惯导报文解析程序、GNSS报文解析程序、控制程序融入到了ROS2中，并且能与MC程序数据互通

[dev_ws-V1.0链接](https://github.com/YuanpengDuan/dev_ws/tree/V1.0)

# dev_ws-V1.1
在V1.0的基础上将寻迹的程序融入到ROS2中，能够正常编译，但是没有测试运行情况

[dev_ws-V1.1链接](https://github.com/YuanpengDuan/dev_ws/tree/V1.1)

# dev_ws-V1.2
测试V1.1中发现程序虽然编译成功但是不能运行下去，代码跑一半在中途退出了。我们的逻辑是先用采点程序采点，pl_node读取采点得到的轨迹文件，然后pl_node读取GNSS传来的GPS数据，与轨迹文件做比较来算出下发给底盘的速度和角度。经过修改，我们成功运行到倒数第二步，最后一步要将计算出来的速度与角度下发给底盘，由于我们没有开小车，只带了GNSS做测试，暂时不知道与底盘的通信情况。

[dev_ws-V1.2链接](https://github.com/YuanpengDuan/dev_ws/tree/V1.2)

# dev_ws-V1.3
测试V1.2过程中发现，之前的采点程序将经纬度除了100，导致记录下来的经纬度是实际经纬度的1/100，修改了此BUG。同时发现程序打印出来的与目标点距离以及计算出来的速度角度始终不变，程序上有问题，待解决。

[dev_ws-V1.3链接](https://github.com/YuanpengDuan/dev_ws/tree/V1.3)

