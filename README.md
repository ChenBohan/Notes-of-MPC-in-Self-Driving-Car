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
      - Reference: 
        - [quadprog](https://www.mathworks.com/help/optim/ug/quadprog.html?s_tid=mwa_osa_a#d120e113143)
        - [Model Predictive Control of a Mobile Robot Using Linearization](http://www.ece.ufrgs.br/~fetter/mechrob04_553.pdf)
      - 解二次规划问题 `x = quadprog(H,f,A,b,Aeq,beq,lb,ub)`
        ```matlab
        H=2*(B'*Q*B+R);
        f=2*B'*Q*A*x_piao(i,:)';
        [X,fval(i,1),exitflag(i,1),output(i,1)]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub);
        ```
      - `XXX`保存每一时刻预测的所有状态值
        ```matlab
        X_PIAO(i,:)=(A*x_piao(i,:)'+B*X)';
        if i+j<Nr
           for j=1:1:Tsim
               XXX(i,1+3*(j-1))=X_PIAO(i,1+3*(j-1))+Xout(i+j,1);
               XXX(i,2+3*(j-1))=X_PIAO(i,2+3*(j-1))+Xout(i+j,2);
               XXX(i,3+3*(j-1))=X_PIAO(i,3+3*(j-1))+Xout(i+j,3);
           end
        else
           for j=1:1:Tsim
               XXX(i,1+3*(j-1))=X_PIAO(i,1+3*(j-1))+Xout(Nr,1);
               XXX(i,2+3*(j-1))=X_PIAO(i,2+3*(j-1))+Xout(Nr,2);
               XXX(i,3+3*(j-1))=X_PIAO(i,3+3*(j-1))+Xout(Nr,3);
           end
        end
        ```
    - 将第一个解施加在系统上
      - `u_piao`保存解出的第一个u
        ```matlab
        u_piao(i,1)=X(1,1);
        u_piao(i,2)=X(2,1);
        ```
      - 将第一个解施加在系统上
        ```matlab
        u_real(i,1)=vd1+u_piao(i,1);
        u_real(i,2)=vd2+u_piao(i,2);
        ```
      - 用解得的控制量更新下一时刻状态
        ```matlab
        XOUT=dsolve('Dx-vd11*cos(z)=0','Dy-vd11*sin(z)=0','Dz-vd22=0','x(0)=X00(1)','y(0)=X00(2)','z(0)=X00(3)');
        t=T; 
        x_real(i+1,1)=eval(XOUT.x);
        x_real(i+1,2)=eval(XOUT.y);
        x_real(i+1,3)=eval(XOUT.z);
        if(i<Nr)
         x_piao(i+1,:)=x_real(i+1,:)-Xout(i+1,:);
        end
        ```
