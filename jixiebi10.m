%% 说明
% 此程序用来对双连杆机械臂进行神经网络控制训练，
% 将得到神经网络训练过程的权值更新图，跟踪误差图，控制输入图，动态拟合图
clear all
close all

%% 机械臂参数
m1 = 0.8;m2 = 2.3; l1 = 1;l2 = 1;g = 10;K =[m1 m2 l1 l2 g];
x0 = [0 0.5 0 0]';
T = 100;               % simulation time    (second)
h = 0.01;              % sampling interval  (second)
N = T/h;               % simulation steps
n = 2;                 % system order
%x0 = [ref1(1,h) ref2(1,h) ref11(1,h) ref21(1,h)]';
clear  m1 m2 l1 l2 g

%% 控制系统参数
Kv = 30; %状态反馈系数
beta1 = 1; %角速度系数
beta2 = 1;

%% 神经网络参数
%神经网络模型建立 
%给出神经元宽度n1,神经元个数N1，神经元位置E1,初始权向量W(1,:)和W（2,:),权值更新系数gama
N1=6*6*6*6;  %神经元个数
n1 = 0.4*1.25;  %神经元宽度
%W1 = zeros(N1,1);
e11 = -1:0.4:1;  %布点范围
e12 = -1:0.4:1;
e13 = -1:0.4:1;
e14 = -1:0.4:1;
W1(1,:) = 0*ones(N1,1);   %初始神经网络权值
W2(1,:) = 0*ones(N1,1); 
gama = 15;  %权值更新系数
diata = 0.001;     %权值更新系数
index = 1;
for i=1:6
    for j=1:6
        for m=1:6
            for k=1:6
                    E1(index, :) = [e11(i) e12(j) e13(m) e14(k)];
                    index = index +1; 
            end
        end
    end
end
clear i j k m index e11 e12 e13 e14;

%% 神经网络训练
x(:,1) = x0;
for k=1:N
    k   
    % 计算控制量
    S = ex(x(:,k),E1,n1,N1);
    yd1(k) = ref1(k,h);
    yd2(k) = ref2(k,h);
    yd11(k) = ref11(k,h);
    yd21(k) = ref21(k,h);
    e1(:,k) = x(1:2,k) - [yd1(k);yd2(k)];
    e2(:,k) = x(3:4,k) - [yd11(k);yd21(k)];
    e(:,k) = beta1*e1(:,k)+beta2*e2(:,k); 
    a(k,:) = [W1(k,:)*S;W2(k,:)*S];
    v1(k) = -Kv*e(1,k)+W1(k,:)*S;
    v2(k) = -Kv*e(2,k)+W2(k,:)*S;
    
    %计算真实动态
    [D H] = dongtai(x(:,k),K);
    FG(k,:) = -H+D*([yd1(k);yd2(k)]+beta2*e2(:,k));
    % 权值更新计算
     u(:,k) = [v1(k);v2(k)];
     
    x(:,k+1) = jixie(x(:,k),u(:,k),h,K);
    yd1(k+1) = ref1(k+1,h);
    yd2(k+1) = ref2(k+1,h);
    yd11(k+1) = ref11(k+1,h);
    yd21(k+1) = ref21(k+1,h);
    e1(:,k+1) = x(1:2,k+1) - [yd1(1+k);yd2(k+1)];
    e2(:,k+1) = x(3:4,k+1) - [yd11(k+1);yd21(k+1)];
    e(:,k+1) = beta1*e1(:,k+1)+beta2*e2(:,k+1); 
    
    W1(k+1,:) = 1*W1(k,:)- h*gama*S'*(e(1,k+1));
    W2(k+1,:) = 1*W2(k,:)- h*gama*S'*(e(2,k+1));
end


%% 训练过程绘图 
%    figure
%    plot(x(1:2,:)','-')
%    axis([-inf inf -0.8 0.8])
%    figure
%    plot(1:length(yd1),yd1',1:length(yd1),yd2,'-')
   
   s = 0:0.01:T;
   figure
   title('权值1更新图')
   plot(s,W1(:,1:100),'-')
   axis([-inf inf -0.5 6])
   xlabel('Time(sec)')
   ylabel('\it{$\hat{W_{1}}$}','interpreter','latex')
   figure
   title('权值2更新图')
   plot(s,W2(:,1:100),'-')
   axis([-inf inf -0.5 2.5])
   xlabel('Time(sec)')
   ylabel('\it{$\hat{W_{2}}$}','interpreter','latex')
   figure
   plot(x(1,:)',x(2,:)','--')
   hold on
   plot(yd1,yd2,'-')
   xlabel('Time(sec)')
%    figure
%    plot(x(3,:)',x(4,:)','-')
%    hold on
%    plot(yd11,yd21,'-')
   s = 0.01:0.01:T;
   figure
   plot(s,u(1,:)','-b',s,u(2,:)','-.r')
   legend('\tau_{1}','\tau_{2}')
  % title('控制输入图')
   xlabel('Time(sec)')
   ylabel('\tau(N)')
   axis([0 50 -inf inf])
   figure
   plot(s,a(:,1),'-b',s,-FG(:,1),'-.r')
   legend({'$\hat{W_{1}}^{T}S(Z)$'},'interpreter','latex','$\tau_{1}^{*}$');
  % title('动态拟合图1')
   xlabel('Time(sec)')
   axis([0 50 -inf inf])
   figure
   plot(s,a(:,2),'-b',s,-FG(:,2),'-.r')
   legend({'$\hat{W_{2}}^{T}S(Z)$'},'interpreter','latex','$\tau_{2}^{*}$');
  % title('动态拟合图2')
   xlabel('Time(sec)')
   axis([0 50 -inf inf])
   figure
   plot(s,e1(1,1:end-1)','-')
   axis([0 50 -1 1])
   xlabel('Time(sec)')
   ylabel('{\it{q}}_1-{\it{q}}_{{\it{d}}1}(rad)')
   %legend('q1-yd1','q2-yd2')
   %title('q1跟踪误差图')
   figure
   plot(s,e1(2,1:end-1)','-')
   axis([0 50 -1 1])
    xlabel('Time(sec)')
    ylabel('{\it{q}}_2-{\it{q}}_{{\it{d}}2}(rad)')
   %legend('q1-yd1','q2-yd2')
  % title('q的跟踪误差图')
 
   
%% 相关函数
%机械臂系统函数
function x1 = jixie (x,u,h,K)
    k1 = mf(x,u,K);
    k2 = mf(x + k1*h/2,u,K);
    k3 = mf(x + k2*h/2,u,K);
    k4 = mf(x + h*k3,u,K);
    x1 = x + h*(k1 + 2*k2 + 2*k3 + k4)/6;
end

function dx = mf(x,u,K)
       m1 = K(1);m2 = K(2); L1 = K(3);L2 = K(4);g = K(5);
       a = (m1+m2)*L1^2; b = m2*L2^2; c = m2*L1*L2; d = g/L1;
       D11 = a + b + 2*c*cos(x(2)); D12 = b + c*cos(x(2));
        D22 = b;
        D = [D11,D12;D12,D22];
        H1 = -c*(2*x(3)*x(4)+x(4)^2)*sin(x(2)) + a*d*cos(x(1)) + c*d*cos(x(1)+x(2));
        H2 = c*x(3)^2*sin(x(2)) + c*d*cos(x(1)+x(2));
        F = [0.2*x(3);0.2*x(4)];
        d1 = 4*[0.1*x(1)+0.15*x(1)*x(2)+0.1*x(3)*x(4);0.1*x(1)+0.2*x(1)*x(2)+0.1*x(3)*x(4)];
        H = 1*[H1;H2]+F;
        dx = [x(3:4);D\(u - H)];
end

%动态计算函数
function  [D,H] = dongtai(x,K)
        m1 = K(1);m2 = K(2); L1 = K(3);L2 = K(4);g = K(5);
        a = (m1+m2)*L1^2; b = m2*L2^2; c = m2*L1*L2; d = g/L1;
        D11 = a + b + 2*c*cos(x(2)); D12 = b + c*cos(x(2));
        D22 = b;
        D = [D11,D12;D12,D22];
        H1 = -c*(2*x(3)*x(4)+x(4)^2)*sin(x(2)) + a*d*cos(x(1)) + c*d*cos(x(1)+x(2));
        H2 = c*x(3)^2*sin(x(2)) + c*d*cos(x(1)+x(2));
        F = [0.2*x(3);0.2*x(4)];
        d1 = 4*[0.1*x(1)+0.15*x(1)*x(2)+0.1*x(3)*x(4);0.1*x(1)+0.2*x(1)*x(2)+0.1*x(3)*x(4)];
        H = 1*[H1;H2]+F;
end

% 计算角度参考信号
function yr = ref1(k,h)
yr = 0.7*sin(k*h)+0.1*cos(k*h);
end

function yr = ref2(k,h)
yr = 0.5*cos(k*h)+0.1*sin(k*h);
end

% 计算角速度参考信号
function yr = ref11(k,h)
yr = 0.7*cos(k*h)-0.1*sin(k*h);
end

function yr = ref21(k,h)
yr = -0.5*sin(k*h)+0.1*cos(k*h);
end

%计算高斯函数向量S
function  S = ex(x,E1,n1,N1)
     for i = 1:N1
        a(i) = sum((x'-E1(i,:)).^2)/(n1.^2);
        S(i,:) = exp(-a(i));
     end 
end
