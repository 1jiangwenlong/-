%% ˵��
% �˳��������Ի�е�۱ջ�����ϵͳ����ѵ�����״̬�۲��������й��ϼ������룬�õ����������Գ���jixiebi14
%  Ԥ�����ʱ��Ϊ6.2s��ʵ�ʼ��ʱ��Ϊ5.6s

clearvars -except fg0 fg1 fg2 xx0  xx1 xx2 W_1 W_2 S0 S1 S2 afa FG0 x00 x11 x22 x33 x44 x55 x66 W1 W2 xxxx1 xxxx2 xxxx3 xxxx4 xxxx5 xxxx6
close all
%  xxx1 = [xxzz(1:end,:);xxgg1(1:end,:)];
%  xxx2 = [xxzz(1:end,:);xxgg2(1:end,:)];
%������Ԫ���n1,��Ԫ����N1����Ԫλ��E1,��ʼȨ����W(1,:)��W��2,:),Ȩֵ����ϵ��gama
N1=5*5*5*5;  %��Ԫ����
n1 = 0.5*1.25;  %��Ԫ���
%W1 = zeros(N1,1);
e11 = -1:0.5:1;  %���㷶Χ
e12 = -1:0.5:1; 
e13 = -1:0.5:1; 
e14 = -1:0.5:1; 
W10=  0*ones(N1,1);   %��ʼ������Ȩֵ
W20 = 0*ones(N1,1); 
afa = 0.3;gama = 1.5;   %������ϲ������������ϵ��
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

%% ���ϼ��
for kkk = 1:4
    switch kkk
        case 1 
            xxx = xxxx1;
        case 2
            xxx = xxxx2;
        case 3
            xxx = xxxx3;
        case 4
            xxx = xxxx4;
        case 5 
            xxx = xxxx5;
        case 6
            xxx = xxxx6;
    end
%   yy1 = identifiers(xxx,afa,[W_1(1,:);W_2(1,:)],E1,n1,N1);
%   ee = yy1 - xxx(:,3:4) ;
 % ee = [eem(1:500,:);eem(510:1000,:)];
%   for kk = 1:2
% %   figure
% %   plot(ee(:,kk))
% %   axis([-inf inf -0.1 0.1])
%  % line([0 length(ee(:,kk))],[T2UCL1,T2UCL1],'LineStyle','--','Color','r');
%    a = abs(ee(:,kk));
% %    figure
% %    plot(a)
% %    axis([-inf inf -0.1 0.1])
%   
%   for i = 1:1:length(a)-80
%       L(i/1) = mean(a(i:i+60));
%   end
%   figure
%   plot(0.1+6:0.1:0.1*length(L(1:end))+6,L(1:end),'-')
%   hold on
% %   plot(0.1+46:0.1:0.1*length(L(551:end))+46,L(551:end),'-')
% %   hold on
%   axis([40 100 -0.01 0.01])
%   %line([0,length(L)],[0.001,0.001],'LineStyle','--','Color','r');
%   %title(['��x',num2str(kk),'��������',num2str(kkk)])
%   
%   line([6,0.1*length(L)+6],[0.001,0.001],'LineStyle','--','Color','r');
%   legend({'$\widetilde{v_{1}}(rad/s)$'},'interpreter','latex','������')
%   end
% %   legend('||q1||','||q2||','������')
% %   title(['������',num2str(kkk)])
%% ���ϸ���

%yy1 = identifiers(xxx,afa,[W_1(1,:);W_2(1,:)],E1,n1,N1);
yy2 = identifiers(xxx,afa,[W_1(2,:);W_2(2,:)],E1,n1,N1);
yy3 = identifiers(xxx,afa,[W_1(3,:);W_2(3,:)],E1,n1,N1);
yy4 = identifiers(xxx,afa,[W_1(4,:);W_2(4,:)],E1,n1,N1);
yy5 = identifiers(xxx,afa,[W_1(5,:);W_2(5,:)],E1,n1,N1);
% yy6 = identifiers(xxx,afa,[W_1(6,:);W_2(6,:)],E1,n1,N1);
% yy7 = identifiers(xxx,afa,[W_1(7,:);W_2(6,:)],E1,n1,N1);
%ee1 = abs(yy1 - xxx(:,3:4)) ;
ee2 = abs(yy2 - xxx(:,3:4)) ;
ee3 = abs(yy3 - xxx(:,3:4)) ;
ee4 = abs(yy4 - xxx(:,3:4)) ;
ee5 = abs(yy5 - xxx(:,3:4)) ;
% ee6 = abs(yy6 - xxx(:,3:4)) ;
% ee7 = abs(yy7 - xxx(:,3:4)) ;
% figure
% plot(ee1(:,2))
% hold on
% plot(ee2(:,2))
% hold on
% plot(ee3(:,2))
% legend('ee1','ee2','ee3')
% axis([-inf inf -0.1 0.1])

a = ee2(:,1);
for kk = 1:2
    
% for i = 1:1:length(a)-80
%       L1(i/1) = mean(ee1(i:i+60,kk));
% end

for i = 1:1:length(a)-80
      L2(i/1) = mean(ee2(i:i+60,kk));
end

for i = 1:1:length(a)-80
      L3(i/1) = mean(ee3(i:i+60,kk));
end
      
for i = 1:1:length(a)-80
      L4(i/1) = mean(ee4(i:i+60,kk));
end

for i = 1:1:length(a)-80
      L5(i/1) = mean(ee5(i:i+60,kk));
end

% for i = 1:1:length(a)-80
%       L6(i/1) = mean(ee6(i:i+60,kk));
% end
% 
% for i = 1:1:length(a)-80
%       L7(i/1) = mean(ee7(i:i+60,kk));
% end

  figure
%   plot(0.1+6:0.1:0.1*length(L1)+6,L1,'-')
%   hold on 
  plot(0.1+6:0.1:0.1*length(L2)+6,L2,'-b')
  hold on
  plot(0.1+6:0.1:0.1*length(L3)+6,L3,'-.r')
  hold on
  plot(0.1+6:0.1:0.1*length(L4)+6,L4,'--k')
  hold on 
  plot(0.1+6:0.1:0.1*length(L5)+6,L5,':m')
%   hold on
%   plot(0.1+6:0.1:0.1*length(L6)+6,L6,'-')
%   hold on
%   plot(0.1+6:0.1:0.1*length(L7)+6,L7,'-')
  lgd = legend('����ģʽd_{1}','����ģʽd_{2}','����ģʽd_{3}','����ģʽd_{4}');%,'����ģʽ5','����ģʽ6')
  lgd.NumColumns = 2;
  %title(['��x',num2str(kk),'���������',num2str(kkk)])
  axis([50 70 -0.01 0.01])
  xlabel('Time(sec)')
end
% yy = identifiers(x1,afa,W,S,fg0);
% yy = identifiers(x1,afa,W,S,fg0);
end
%% ��̬����������

function yy = identifiers(x1,afa,WW,E1,n1,N1)
       [N M] = size(x1);
        %W(1) = struct('WW',WW,'x',x1(1,:));
        yy(1,:) = x1(1,3:4);
        for i=1:N
            S(i,:) = ex(x1(i,:),E1,n1,N1);
            i
        end
        for i=1:N-1
            for j=1:2
               yy(i+1,j) =  x1(i,j+2)+afa*(yy(i,j)-x1(i,j+2))+0.0+0.1*WW(j,:)*S(i,:)';
            end
            i
        end  
end

%% ��غ���
%�����˹��������S
function  S = ex(x,E1,n1,N1)
     for i = 1:N1
        a(i) = sum((x-E1(i,:)).^2)/(n1.^2);
        S(i,:) = exp(-a(i));
     end 
end