# Self-Driving-Car-MPC-01-MPC-Basic
《无人驾驶车辆模型预测控制》- 龚建伟 - 第三章 - 模型预测控制算法基础与仿真分析

### chapter3-3-3.m

- 参考轨迹生成
  - Xout
    - 第一列为每个时刻`T, 2T, ..., nT`
    - 第二列为参考轨迹`y = 2`
    
- 控制系统基本情况
  - 3个状态量:`x速度，y速度，角速度`
  - 2个控制量:`线速度、角速度`
  
- 矩阵定义
  - `x_real` 记录车辆的观测坐标
    - `x_real`第1行填初始位置 `x_real(1,:)=X0;`（`X0 = [0 0 pi/3];`）
  - `x_piao` 记录车辆当前坐标与参考坐标的差
    - `x_piao(1,:)=x_real(1,:)-Xout(1,:);`
  - `u_real=zeros(Nr,2); u_piao=zeros(Nr,2)` 
  
- MPC主体
  - 每一个周期`for i=1:1:Nr`
    - 获取系统当前状态量、更新状态空间方程
      - 用系统当前状态（朝向）更新矩阵a、b
        ```matlab
        t_d =Xout(i,3);
        a=[1    0   -vd1*sin(t_d)*T;
           0    1   vd1*cos(t_d)*T;
           0    0   1;];
        b=[cos(t_d)*T   0;
           sin(t_d)*T   0;
           0            T;];    
        ```
    - 求解一个标准二次规划问题    
      ```matlab
      [X,fval(i,1),exitflag(i,1),output(i,1)]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub);
      ```
    - 将第一个解施加在系统上
  - 
    
