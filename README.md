# myPlanners

## 展示几个硕士期间做的规划算法，环境为matlab，二次规划问题使用matlab自带求解函数quadprog，非线性规划问题部分使用到插件[YALMIP](https://github.com/yalmip/YALMIP)和求解器[IPOPT](https://github.com/coin-or/Ipopt)


## 侧方位停车（倒序）

![image](https://github.com/Msc-70/myPlanners/raw/master/videos/trajectory_plot_case2.png)





![image](https://github.com/Msc-70/myPlanners/raw/master/videos/parallel_case1_.gif)


分为入库和从车位外到入库起始点两阶段分别进行轨迹规划，由于两阶段非线性规划问题的设置需要，停车的终点即车位内停靠点被设定为起点，而车位以外的特定位置被设定为终点




## 规则道路轨迹规划

![image](https://github.com/Msc-70/myPlanners/raw/master/videos/demo_.gif)



采用速度-路径解耦的轨迹生成方式，B样条曲线模型分别生成平滑的速度曲线和参考路径曲线，二者以累计长度作为匹配点组合成为时空轨迹，在轨迹评价和碰撞检测后选择最优的一条作为最后的行驶参考，规划周期为0.2s
