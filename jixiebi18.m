%% 说明
% 此程序用来得到机械臂闭环控制系统（神经网络控制，即采用jixiebi程序的控制方法）在某个时间点发生故障的连续数据

clearvars -except fg0 fg1 fg2 xx0  xx1 xx2 W_1 W_2 S0 S1 S2 afa FG0 x00  x11  x22 x33 x44 x55 x66 W1 W2 xxx1 xxx2 xxx3 xxx4 xxx5 xxx6 E1 n1 N1
close all
%% 机械臂参数
m1 = 0.8;m2 = 2.3; l1 = 1;l2 = 1;g = 10;K =[m1 m2 l1 l2 g];
%x0 = [ref1(1,h) ref2(1,h) ref11(1,h) ref21(1,h)]';
T = 100;               % simulation time    (second)
h = 0.01;              % sampling interval  (second)
N = T/h;               % simulation steps
n = 2;                 % system order
x0 = [ref1(1,h) ref2(1,h) ref11(1,h) ref21(1,h)]';
clear  m1 m2 l1 l2 g

%% 控制系统参数
Kv = 20; %状态反馈系数
beta1 = 1; %角速度系数
beta2 = 1;

%% 控制部分的神经网络参数
%神经网络模型建立 
%给出神经元宽度n1,神经元个数N1，神经元位置E1,初始权向量W(1,:)和W（2,:),权值更新系数gama
N1=6*6*6*6;  %神经元个数
n1 = 0.4*1.25;  %神经元宽度
%W1 = zeros(N1,1);
e11 = -1:0.4:1;  %布点范围
e12 = -1:0.4:1;
e13 = -1:0.4:1;
e14 = -1:0.4:1;
W10= 0*ones(N1,1);   %初始神经网络权值
W20 = 0*ones(N1,1); 
afa = 0.1;gama = 0.1;   %故障诊断部分神经网络更新系数
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

%% 计算正常和故障系统状态 
   [FGz,xz] = sys(x0,W1,W2,E1,n1,N1,0.5*N,h,beta1,beta2,Kv,K,0,0);
   xxz = xz';nn = 0.5*N;
   [FGg,xg] = sys(xxz(end,:)',W1,W2,E1,n1,N1,0.5*N,h,beta1,beta2,Kv,K,1,nn);
   xxg1 = xg';
   [FGg,xg] = sys(xxz(end,:)',W1,W2,E1,n1,N1,0.5*N,h,beta1,beta2,Kv,K,2,nn);
   xxg2 = xg';
   [FGg,xg] = sys(xxz(end,:)',W1,W2,E1,n1,N1,0.5*N,h,beta1,beta2,Kv,K,3,nn);
   xxg3 = xg';
   [FGg,xg] = sys(xxz(end,:)',W1,W2,E1,n1,N1,0.5*N,h,beta1,beta2,Kv,K,4,nn);
   xxg4 = xg';
%    [FGg,xg] = sys(xxz(end,:)',W1,W2,E1,n1,N1,0.5*N,h,beta1,beta2,Kv,K,5,nn);
%    xxg5 = xg';
%    [FGg,xg] = sys(xxz(end,:)',W1,W2,E1,n1,N1,0.5*N,h,beta1,beta2,Kv,K,6,nn);
%    xxg6 = xg';
%    %xxx2 = [xxz;xxg2];
%    for i = 10:10:5000
%     xxzz(i/10,:) = xxz(i,:);
%    end
%    for i = 10:10:5000
%     xxgg1(floor(i/10)+1,:) = xxg1(i,:);
%    end
%    for i = 10:10:5000
%     xxgg2(floor(i/10)+1,:) = xxg2(i,:);
%    end
   xxx1 = [xxz(1:end,:);xxg1(2:end-1,:)];
   xxx2 = [xxz(1:end,:);xxg2(2:end-1,:)];
   xxx3 = [xxz(1:end,:);xxg3(2:end-1,:)];
   xxx4 = [xxz(1:end,:);xxg4(2:end-1,:)];
%    xxx5 = [xxz(1:end,:);xxg5(2:end-1,:)];
%    xxx6 = [xxz(1:end,:);xxg6(2:end-1,:)];
   for i = 10:10:length(xxx1)
       xxxx1(i/10,:) = xxx1(i,:);
   end
   for i = 10:10:length(xxx2)
       xxxx2(i/10,:) = xxx2(i,:);
   end
   for i = 10:10:length(xxx3)
       xxxx3(i/10,:) = xxx3(i,:);
   end
   for i = 10:10:length(xxx4)
       xxxx4(i/10,:) = xxx4(i,:);
   end
%    for i = 10:10:length(xxx5)
%        xxxx5(i/10,:) = xxx5(i,:);
%    end
%    for i = 10:10:length(xxx6)
%        xxxx6(i/10,:) = xxx6(i,:);
%    end
   figure
   plot(xxxx1(:,3),xxxx1(:,4),'-')
   hold on
   plot(xxxx2(:,3),xxxx2(:,4),'-')
   hold on
   plot(xxxx3(:,3),xxxx3(:,4),'-')
   hold on
   plot(xxxx4(:,3),xxxx4(:,4),'-')
%    hold on
%    plot(xxxx5(:,3),xxxx5(:,4),'-')
%    hold on
%    plot(xxxx6(:,3),xxxx6(:,4),'-')
   legend('系统发生故障1','系统发生故障2','系统发生故障3','系统发生故障4')%,'系统发生故障5','系统发生故障6')
   xlabel({'$\dot{q_{1}}(rad/s)$'},'interpreter','latex')
   ylabel({'$\dot{q_{2}}(rad/s)$'},'interpreter','latex')
%    figure 
%    plot(xxzz(300:end,3),xxzz(300:end,4),'-r')
%    hold on
%    plot(xxgg1(1,3),xxgg1(1,4),'o')
%    hold on
%    plot(xxgg2(1:end,3),xxgg2(1:end,4),'--g')
%    hold on
%    plot(xxgg1(1:end,3),xxgg1(1:end,4),'--b')
%    figure
%    plot(xxx1(3000:10000,3),xxx1(3000:10000,4),'--')
%    figure
%    plot(xxx2(3000:10000,3),xxx2(3000:10000,4),'--')
%    figure 
%    plot(xxz(3000:end,3),xxz(3000:end,4),'-r')
%    hold on
%    plot(xxg2(1,3),xxg2(1,4),'o')
%    hold on
%    plot(xxg2(1:end,3),xxg2(1:end,4),'--b')
%    [FG1,x] = sys(x0,W1,W2,E1,n1,N1,N,h,beta1,beta2,Kv,K,1);
%    x11 = x';
%    [FG2,x] = sys(x0,W1,W2,E1,n1,N1,N,h,beta1,beta2,Kv,K,2);
%    x22 = x';
   
%% 相关函数

% 系统状态计算函数
function [FG,x] = sys(x0,W1,W2,E1,n1,N1,N,h,beta1,beta2,Kv,K,i,nn)
  W11 = mean(W1(9500:10000,:));
  W22 = mean(W2(9500:10000,:)); %学习后的神经网络权值
  
  x(:,1) = x0;
for k=1:N
    k   
    % 计算控制量
    S = ex(x(:,k),E1,n1,N1);
    yd1(k) = ref1(k+nn,h);
    yd2(k) = ref2(k+nn,h);
    yd11(k) = ref11(k+nn,h);
    yd21(k) = ref21(k+nn,h);
    e1(:,k) = x(1:2,k) - [yd1(k);yd2(k)];
    e2(:,k) = x(3:4,k) - [yd11(k);yd21(k)];
    e(:,k) = beta1*e1(:,k)+beta2*e2(:,k); 
    a(k,:) = [W11*S;W22*S];
    v1(k) = -Kv*e(1,k)+W11*S;
    v2(k) = -Kv*e(2,k)+W22*S;
    
    % 状态更新计算
    u(:,k) = [v1(k);v2(k)];    
    x(:,k+1) = jixie(x(:,k),u(:,k),h,K,i);   
    %计算系统已知部分,即为FG
    [D H] = dongtai(x(:,k),K);
    FG(k,:) = D\(u(:,k)-H); 
end
%    figure
%    plot(u','-')
%    legend('v1','v2')
%    title('控制输入图')
%    figure
%    plot(1:length(a(:,1)),a(:,1),1:length(a(:,1)),-FG(:,1),'-')
%    legend('W1*S','FG1')
%    title('动态拟合图1')
%    figure
%    plot(1:length(a(:,2)),a(:,2),1:length(a(:,2)),-FG(:,2),'-')
%    legend('W2*S','FG2')
%    title('动态拟合图2')
%    figure
%    plot(e1','-')
%    axis([-inf inf -1 1])
%    legend('q1-yd1','q2-yd2')
%    title('q的跟踪误差图')
%    figure
%    plot(x(3,:)',x(4,:)','-')
%    hold on
%    plot(yd11,yd21,'-')
end

%机械臂系统函数
function x1 = jixie (x,u,h,K,i)
    k1 = mf(x,u,K,i);
    k2 = mf(x + k1*h/2,u,K,i);
    k3 = mf(x + k2*h/2,u,K,i);
    k4 = mf(x + h*k3,u,K,i);
    x1 = x + h*(k1 + 2*k2 + 2*k3 + k4)/6;
end

function dx = mf(x,u,K,i)
       m1 = K(1);m2 = K(2); L1 = K(3);L2 = K(4);g = K(5);
       a = (m1+m2)*L1^2; b = m2*L2^2; c = m2*L1*L2; d = g/L1;
       D11 = a + b + 2*c*cos(x(2)); D12 = b + c*cos(x(2));
        D22 = b;
        D = [D11,D12;D12,D22];
        H1 = -c*(2*x(3)*x(4)+x(4)^2)*sin(x(2)) + a*d*cos(x(1)) + c*d*cos(x(1)+x(2));
        H2 = c*x(3)^2*sin(x(2)) + c*d*cos(x(1)+x(2));
        F = [0.2*x(3);0.2*x(4)];
        d1 = 10*[0.1*x(1)+0.15*x(1)*x(2)+0.1*x(3)*x(4);0.1*x(1)+0.2*x(1)*x(2)+0.1*x(3)*x(4)];
        switch i
            case 0
                H = 1*[H1;H2]+F;
            case 1
                H = 1.05*[H1;H2]+F;
            case 2
                H = 1.1*[H1;H2]+F;
%             case 3
%                 H = 1.15*[H1;H2]+F;
            case 3
                H = 1.00*[H1;H2]+F+d1;
            case 4
                H = 1.00*[H1;H2]+F+1.5*d1;
%             case 6
%                 H = 1.00*[H1;H2]+F+2*d1;   
        end
        dx = [x(3:4);D\(u - H)];
end

%已知系统部分计算函数
function  [D,H] = dongtai(x,K)
        m1 = K(1);m2 = K(2); L1 = K(3);L2 = K(4);g = K(5);
        a = (m1+m2)*L1^2; b = m2*L2^2; c = m2*L1*L2; d = g/L1;
        D11 = a + b + 2*c*cos(x(2)); D12 = b + c*cos(x(2));
        D22 = b;
        D = [D11,D12;D12,D22];
        H1 = -c*(2*x(3)*x(4)+x(4)^2)*sin(x(2)) + a*d*cos(x(1)) + c*d*cos(x(1)+x(2));
        H2 = c*x(3)^2*sin(x(2)) + c*d*cos(x(1)+x(2));
        F = [0.2*x(3);0.2*x(4)];
        d1 = 10*[0.1*x(1)+0.15*x(1)*x(2)+0.1*x(3)*x(4);0.1*x(1)+0.2*x(1)*x(2)+0.1*x(3)*x(4)];
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
