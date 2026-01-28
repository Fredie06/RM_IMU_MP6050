知乎链接：https://zhuanlan.zhihu.com/p/1999886285305835642
# RM_IMU_BMI088
BMI088 IMU attitude estimate on RT-Thread + RoboMaster C-board   在 RT-Thread 下实现互补滤波与Madgwick姿态解算
## 效果演示
![alt text](IMU.gif)
## 快速开始
### Madgwick算法：
将本仓库的main.c直接替换BSP包的application中的main.c文件，兼容DJI开发板C型

### 互补滤波算法：
将Complementary_Filter_Algorithm.c文件中的代码直接替换main.c中的mahony_ahrs_updateIMU函数内容即可
## 硬件/依赖
DJI开发板C型
## 软件架构
main.c              # 线程调度以及Madgwick算法实现姿态解算

IMU.gif           # 效果展示动图

Complementary_Filter_Algorithm.c #互补滤波算法补丁
