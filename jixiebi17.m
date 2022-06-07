%% 说明
% 此程序用来对机械臂闭环控制系统进行状态观测器的权值训练，用到的数据来自程序jixiebi12
% 并将训练好的神经网络再次用于机械臂状态观测中

clearvars -except  x00  x11  x22  x33 x44 x55 x66 FG0 FG1 FG2 FG3 FG4 FG5 FG6 W1 W2 xxx1 xxx2
close all
T = 9200:10:10000;
figure
% plot(x00(T,3),x00(T,4),'-')
% hold on
plot(x11(T,3),x11(T,4),'-b')
hold on
plot(x22(T,3),x22(T,4),'-.r')
hold on
plot(x33(T,3),x33(T,4),'--k')
hold on
plot(x44(T,3),x44(T,4),':m')
hold on
% plot(x55(T,3),x55(T,4),'-')
% hold on
% plot(x66(T,3),x66(T,4),'-')
 legend('故障模式d_{1}','故障模式d_{2}','故障模式d_{3}','故障模式d_{4}')%,'系统发生故障5','系统发生故障6')
xlabel({'$\dot{q_{1}}(rad/s)$'},'interpreter','latex')
ylabel({'$\dot{q_{2}}(rad/s)$'},'interpreter','latex')
%绘制系统图形
% figure
% plot(FG0,'-')
% axis([-inf inf -2.5 2.5])
% figure
% plot(FG1,'-')
% axis([-inf inf -2.5 2.5])
% figure
% plot(FG2,'-')
% axis([-inf inf -2.5 2.5])
% figure
% plot(x00,'-')
% legend('q1','q2','q3','q4')
% figure
% plot(x11,'-')
% legend('q1','q2','q3','q4')
% figure
% plot(x22,'-')
%legend('q1','q2','q3','q4')
for i = 10:10:10000
    xx0(i/10,:) = x00(i,:);
end
for i = 10:10:10000
    xx1(i/10,:) = x11(i,:);
end
for i = 10:10:10000
    xx2(i/10,:) = x22(i,:);
end
for i = 10:10:10000
    xx3(i/10,:) = x33(i,:);
end
for i = 10:10:10000
    xx4(i/10,:) = x44(i,:);
end
% for i = 10:10:10000
%     xx5(i/10,:) = x55(i,:);
% end
% for i = 10:10:10000
%     xx6(i/10,:) = x66(i,:);
% end
for i = 10:10:10000
    fg0(i/10,:) = FG0(i,:);
end
for i = 10:10:10000
    fg1(i/10,:) = FG1(i,:);
end
for i = 10:10:10000
    fg2(i/10,:) = FG2(i,:);
end
for i = 10:10:10000
    fg3(i/10,:) = FG3(i,:);
end
for i = 10:10:10000
    fg4(i/10,:) = FG4(i,:);
end
% for i = 10:10:10000
%     fg5(i/10,:) = FG5(i,:);
% end
% for i = 10:10:10000
%     fg6(i/10,:) = FG6(i,:);
% end
% figure
% plot(xx0,'-')
% legend('q1','q2','q3','q4')
%% 控制部分的神经网络参数
%神经网络模型建立 
%给出神经元宽度n1,神经元个数N1，神经元位置E1,初始权向量W(1,:)和W（2,:),权值更新系数gama
N1=5*5*5*5;  %神经元个数
n1 = 0.5*1.25;  %神经元宽度
%W1 = zeros(N1,1);
e11 = -1:0.5:1;  %布点范围
e12 = -1:0.5:1; 
e13 = -1:0.5:1; 
e14 = -1:0.5:1; 
W10=  0*ones(N1,1);   %初始神经网络权值
W20 = 0*ones(N1,1); 
afa = 0.3;gama = 1.5;   %故障诊断部分神经网络更新系数
index = 1;
for i=1:5
    for j=1:5
        for m=1:5
            for k=1:5
                    E1(index, :) = [e11(i) e12(j) e13(m) e14(k)];
                    index = index +1; 
            end
        end
    end
end

%% 神经网络训练
%T = 1:10000;
%[S0,W00] = neu_train(xx0,W10,W20,fg0,E1,n1,N1,afa,gama);
[S0,W00] = neu_train(xx0,W10,W20,fg1,E1,n1,N1,afa,gama);
[S1,W11] = neu_train(xx1,W10,W20,fg1,E1,n1,N1,afa,gama);
[S2,W22] = neu_train(xx2,W10,W20,fg2,E1,n1,N1,afa,gama);
[S3,W33] = neu_train(xx3,W10,W20,fg3,E1,n1,N1,afa,gama);
[S4,W44] = neu_train(xx4,W10,W20,fg4,E1,n1,N1,afa,gama);
% [S5,W55] = neu_train(xx5,W10,W20,fg5,E1,n1,N1,afa,gama);
% [S6,W66] = neu_train(xx6,W10,W20,fg6,E1,n1,N1,afa,gama);
for kk = 0:4
    switch kk
        case 0
            W = W00;XX = xx0(:,3:4);
        case 1
            W = W11;XX = xx1(:,3:4);
        case 2
            W = W22;XX = xx2(:,3:4);
        case 3
            W = W33;XX = xx3(:,3:4);
        case 4
            W = W44;XX = xx4(:,3:4);
%         case 5
%             W = W55;XX = xx5(:,3:4);
%         case 6
%             W = W66;XX = xx6(:,3:4);
    end
for i = 1:length(W)
    WW11(i,:) = W(i).WW(1,:);
    WW21(i,:) = W(i).WW(2,:);
end
W_1(kk+1,:) = mean(WW11(900:end,:));
W_2(kk+1,:) = mean(WW21(900:end,:));
for i = 1:length(W)
    i
    ee(i,:) = W(i).x-XX(i,:);
end
s = 0.1:0.1:100;
figure
plot(s,WW11(1:end,1:end),'-')
xlabel('Time(sec)')
figure
plot(s,WW21(1:end,1:end),'-')
xlabel('Time(sec)')
figure
plot(s,ee(:,:),'-')
xlabel('Time(sec)')
% axis([-inf inf -0.5 0.5])
% legend({'$\hat{W_{1}}^{T}S(Z)$'},'interpreter','latex','$FG_{1}$');
%legend({'$\widetilde{v_{1}}(rad/s)$'},{'$\widetilde{v_{1}}(rad/s)$'},'interpreter','latex')
% figure
% plot(s,ee(:,2),'-')
% axis([-inf inf -0.5 0.5]
% legend({'$$\overline{v_{2}}(rad/s)$$'},'interpreter','latex')
%title('状态观测器的训练过程')
end


   
% figure
% plot(x00(1,T)',x00(2,T)',x00(3,T)','-')
% hold on
% plot(x11(1,T)',x11(2,T)',x11(3,T)','-')
% hold on
% plot(x22(1,T)',x22(2,T)',x22(3,T)','-')
% figure
% subplot(1,2,1)
% plot(W1(:,1:150),'-')
% subplot(1,2,2)
% plot(W2(:,1:150),'-')
% % figure
% % plot(x00(3,T)',x00(4,T)','-')
% % hold on
% % plot(xx(:,1),xx(:,2),'--')
% figure
% plot(W11(9000:,1:100),'-')

%% 神经网络的再次使用
% 用W估计对应的轨迹的图形
for i = 0:4
    switch i
        case 0
           yy = identifiers(xx0,afa,[W_1(1,:);W_2(1,:)],S0,fg0);ee = yy - xx0(:,3:4) ;
        case 1
            yy = identifiers(xx1,afa,[W_1(2,:);W_2(2,:)],S1,fg1);ee = yy - xx1(:,3:4) ;
        case 2
            yy = identifiers(xx2,afa,[W_1(3,:);W_2(3,:)],S2,fg2);ee = yy - xx2(:,3:4) ;
        case 3
           yy = identifiers(xx3,afa,[W_1(4,:);W_2(4,:)],S3,fg3);ee = yy - xx3(:,3:4) ;
        case 4
            yy = identifiers(xx4,afa,[W_1(5,:);W_2(5,:)],S4,fg4);ee = yy - xx4(:,3:4) ;
%         case 5
%             yy = identifiers(xx5,afa,[W_1(6,:);W_2(6,:)],S5,fg5);ee = yy - xx5(:,3:4) ;
%         case 6
%             yy = identifiers(xx6,afa,[W_1(7,:);W_2(7,:)],S6,fg6);ee = yy - xx6(:,3:4) ;
    end
    %ee = yy - xx0(:,3:4) ;
%     s = 0.1:0.1:100;
%     figure
%     plot(s,ee)
%     axis([-inf inf -0.5 0.5])
%     xlabel('Time(sec)')
%     ylabel({'$\widetilde{v}(rad/s)$'},'interpreter','latex')
%     legend({'$\hat{W_{1}}^{T}S(Z)$'},'interpreter','latex','$FG_{1}$');
    %title('神经网络的再次使用（用W估计对应的轨迹的图形）')
end
% 用不同的W估计不同的轨迹
for i = 0:4
    switch i
        case 0
            xx = xx0;S = S0;
        case 1
            xx = xx1;S = S1;   
        case 2
            xx = xx2;S = S2;
        case 3
            xx = xx3;S = S3;
        case 4
            xx = xx4;S = S4;   
%         case 5
%             xx = xx5;S = S5;
%         case 6
%             xx = xx6;S = S6;
    end
    yy0 = identifiers(xx,afa,[W_1(2,:);W_2(2,:)],S,fg1);
    yy1 = identifiers(xx,afa,[W_1(2,:);W_2(2,:)],S,fg1);
    yy2 = identifiers(xx,afa,[W_1(3,:);W_2(3,:)],S,fg1);
    yy3 = identifiers(xx,afa,[W_1(4,:);W_2(4,:)],S,fg1);
    yy4 = identifiers(xx,afa,[W_1(5,:);W_2(5,:)],S,fg1);
%     yy5 = identifiers(xx,afa,[W_1(6,:);W_2(6,:)],S,fg1);
%     yy6 = identifiers(xx,afa,[W_1(7,:);W_2(7,:)],S,fg1);
    ee0 = abs(yy0 - xx(:,3:4)) ;
    ee1 = abs(yy1 - xx(:,3:4)) ;
    ee2 = abs(yy2 - xx(:,3:4)) ;
    ee3 = abs(yy3 - xx(:,3:4)) ;
    ee4 = abs(yy4 - xx(:,3:4)) ;
%     ee5 = abs(yy5 - xx(:,3:4)) ;
%     ee6 = abs(yy6 - xx(:,3:4)) ;
    figure
    plot(ee1(:,1))
    hold on
    plot(ee2(:,1))
    hold on
    plot(ee3(:,1))
    hold on
    plot(ee4(:,1))
%     hold on
%     plot(ee5(:,1))
%     hold on
%     plot(ee6(:,1))
    %legend('ee1','ee2','ee3')
    axis([-inf inf -0.3 0.3])
end
% yy = identifiers(x1,afa,W,S,fg0);
% yy = identifiers(x1,afa,W,S,fg0);
%% 动态估计器函数
function yy = identifiers(x1,afa,WW,S,fg0)
       [N M] = size(x1);
        W(1) = struct('WW',WW,'x',x1(1,:));
        yy(1,:) = x1(1,3:4);
        for i=1:N-1
            for j=1:2
               W(i+1).x(j) =  x1(i,j+2)+afa*(W(i).x(j)-x1(i,j+2))+0.0*fg0(i,j)+0.1*W(1).WW(j,:)*S(i,:)';
               yy(i+1,j) = W(i+1).x(j);
            end
            i
        end  
end
%% 神经网络训练函数
%x1（大小为N*4)为输入状态，
%neu_can为初始神经网络参数，
%x为拟合的状态，
%W为更新过程中权值的所有值，
% function [W1,W2,xx] = neu_train(x1,W10,W20,FG,E1,n1,N1,afa,gama)
%         [N M] = size(x1);
%         xx(1,:) = [0 0];
%         W1(1,:) = W10;W2(1,:) = W20;
%         %计算高斯向量
%         for k=1:N
%             S(k,:) = ex(x1(k,:),E1,n1,N1);
%             k
%         end
%         for i=1:N-1             
%               xx(i+1,1) =  x1(i,3) + afa*(xx(i,1)-x1(i,3))+W1(i,:)*S(i,:)';
%               xx(i+1,2) =  x1(i,4) + afa*(xx(i,2)-x1(i,4))+W2(i,:)*S(i,:)';
%               W1(i+1,:) =  W1(i,:) - gama*S(i,:)*(xx(i+1,1)-x1(i+1,3));
%               W2(i+1,:) =  W2(i,:) - gama*S(i,:)*(xx(i+1,2)-x1(i+1,4));
%               i
%         end  
% end
function [S,W] = neu_train(x1,W10,W20,fg0,E1,n1,N1,afa,gama)
        [N M] = size(x1);
        W(1) = struct('WW',[W10';W20'],'x',[0 0]);
        %计算高斯向量
        for i=1:N
            S(i,:) = ex(x1(i,:),E1,n1,N1);
            i
        end
        for i=1:N-1
            for j=1:2
               W(i+1).x(j) =  x1(i,j+2)+afa*(W(i).x(j)-x1(i,j+2))+0.0*fg0(i,j)+0.1*W(i).WW(j,:)*S(i,:)';
               W(i+1).WW(j,:) =W(i).WW(j,:)-0.1*gama*S(i,:)*(W(i+1).x(j)-x1(i+1,j+2));
            end
            i
        end  
end
%% 相关函数
%计算高斯函数向量S
function  S = ex(x,E1,n1,N1)
     for i = 1:N1
        a(i) = sum((x-E1(i,:)).^2)/(n1.^2);
        S(i,:) = exp(-a(i));
     end 
end