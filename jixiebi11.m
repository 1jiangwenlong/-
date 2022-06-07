%% ˵��
% ��jixiebi����������ѵ����ɺ󣬴˳�����Խ�����������Ȩֵ����������ֵ�������������ƣ�
% ���õ�����ֵ��������Ƶĸ������ͼ����������ͼ����̬���ͼ

close all
clearvars -except W1 W2
%% ��е�۲���
m1 = 0.8;m2 = 2.3; l1 = 1;l2 = 1;g = 10;K =[m1 m2 l1 l2 g];
%x0 = [ref1(1,h) ref2(1,h) ref11(1,h) ref21(1,h)]';
T = 40;               % simulation time    (second)
h = 0.01;              % sampling interval  (second)
N = T/h;               % simulation steps
n = 2;                 % system order
x0 = [ref1(1,h) ref2(1,h) ref11(1,h) ref21(1,h)]';
clear  m1 m2 l1 l2 g

%% ����ϵͳ����
Kv = 20; %״̬����ϵ��
beta1 = 1; %���ٶ�ϵ��
beta2 = 1;

%% ���������
%������ģ�ͽ��� 
%������Ԫ���n1,��Ԫ����N1����Ԫλ��E1,��ʼȨ����W(1,:)��W��2,:),Ȩֵ����ϵ��gama
N1=6*6*6*6;  %��Ԫ����
n1 = 0.4*1.25;  %��Ԫ���
%W1 = zeros(N1,1);
e11 = -1:0.4:1;  %���㷶Χ
e12 = -1:0.4:1;
e13 = -1:0.4:1;
e14 = -1:0.4:1;
W1(1,:) = 0*ones(N1,1);   %��ʼ������Ȩֵ
W2(1,:) = 0*ones(N1,1); 
gama = 50;  %Ȩֵ����ϵ��
diata = 0.001;     %Ȩֵ����ϵ��
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

%% ��������Թ���
  W11 = mean(W1(9000:10000,:));
  W22 = mean(W2(9000:10000,:)); %ѧϰ���������Ȩֵ
  
  x(:,1) = x0;
for k=1:N
    k   
    % ���������
    S = ex(x(:,k),E1,n1,N1);
    yd1(k) = ref1(k,h);
    yd2(k) = ref2(k,h);
    yd11(k) = ref11(k,h);
    yd21(k) = ref21(k,h);
    e1(:,k) = x(1:2,k) - [yd1(k);yd2(k)];
    e2(:,k) = x(3:4,k) - [yd11(k);yd21(k)];
    e(:,k) = beta1*e1(:,k)+beta2*e2(:,k); 
    a(k,:) = [W11*S;W22*S];
    v1(k) = -Kv*e(1,k)+W11*S;
    v2(k) = -Kv*e(2,k)+W22*S;
    
    %������ʵ��̬
    [D H] = dongtai(x(:,k),K);
    FG(k,:) = -H+D*([yd1(k);yd2(k)]+beta2*e2(:,k));
    % ״̬���¼���
     u(:,k) = [v1(k);v2(k)];    
    x(:,k+1) = jixie(x(:,k),u(:,k),h,K);   
end

%% ���Թ��̻�ͼ 
   s = 0.01:0.01:T;
   figure
   plot(s,u(1,:)','-b',s,u(2,:)','-.r')
   legend('\tau_{1}','\tau_{2}')
  % title('��������ͼ')
   xlabel('Time(sec)')
   ylabel('\tau(N)')
   figure
   plot(s,a(:,1),'-b',s,-FG(:,1),'-.r')
   legend({'$\overline{W_{1}}^{T}S(Z)$'},'interpreter','latex','$\tau_{1}$');
  % title('��̬���ͼ1')
  xlabel('Time(sec)')
   
   figure
   plot(s,a(:,2),'-b',s,-FG(:,2),'-.r')
   legend({'$\overline{W_{2}}^{T}S(Z)$'},'interpreter','latex','$\tau_{2}$');
 %  title('��̬���ͼ2')
 xlabel('Time(sec)')
   
   figure
   plot(s,e1(1,:)','-b')
   axis([-inf inf -0.02 0.02])
   xlabel('Time(sec)')
   ylabel('{\it{q}}_1-{\it{q}}_{{\it{d}}1}(rad)')
   %legend('q1-yd1','q2-yd2')
   %title('q�ĸ������ͼ')
   figure
   plot(s,e1(2,:)','-b')
   axis([-inf inf -0.02 0.02])
   xlabel('Time(sec)')
   ylabel('{\it{q}}_2-{\it{q}}_{{\it{d}}2}(rad)')
%% ��غ���
%��е��ϵͳ����
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
        d1 = 4*[0.1*x(1)+0.2*x(1)*x(2)+0.1*x(3)*x(4);0.1*x(1)+0.15*x(1)*x(2)+0.1*x(3)*x(4)];
        H = 1.0*[H1;H2]+F;
        dx = [x(3:4);D\(u - H)];
end

%��̬���㺯��
function  [D,H] = dongtai(x,K)
        m1 = K(1);m2 = K(2); L1 = K(3);L2 = K(4);g = K(5);
        a = (m1+m2)*L1^2; b = m2*L2^2; c = m2*L1*L2; d = g/L1;
        D11 = a + b + 2*c*cos(x(2)); D12 = b + c*cos(x(2));
        D22 = b;
        D = [D11,D12;D12,D22];
        H1 = -c*(2*x(3)*x(4)+x(4)^2)*sin(x(2)) + a*d*cos(x(1)) + c*d*cos(x(1)+x(2));
        H2 = c*x(3)^2*sin(x(2)) + c*d*cos(x(1)+x(2));
        F = [0.2*x(3);0.2*x(4)];
        d1 = 4*[0.1*x(1)+0.2*x(1)*x(2)+0.1*x(3)*x(4);0.1*x(1)+0.15*x(1)*x(2)+0.1*x(3)*x(4)];
        H = 1.0*[H1;H2]+F;
end

% ����ǶȲο��ź�
function yr = ref1(k,h)
yr = 0.7*sin(k*h)+0.1*cos(k*h);
end

function yr = ref2(k,h)
yr = 0.5*cos(k*h)+0.1*sin(k*h);
end

% ������ٶȲο��ź�
function yr = ref11(k,h)
yr = 0.7*cos(k*h)-0.1*sin(k*h);
end

function yr = ref21(k,h)
yr = -0.5*sin(k*h)+0.1*cos(k*h);
end

%�����˹��������S
function  S = ex(x,E1,n1,N1)
     for i = 1:N1
        a(i) = sum((x'-E1(i,:)).^2)/(n1.^2);
        S(i,:) = exp(-a(i));
     end 
end
