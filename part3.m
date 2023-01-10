clear;close all;clc
t=0:0.02:5;s=t/5;
dt=0.02;n=1:251; 
p_i=[100;100];p_f=[0;150]; %初始末端坐标与跟踪路径终点
l=100;m=1;%杆长和质量
%初始化
p(2,251)=0;c2(251)=0;s2(251)=0;c1(251)=0;s1(251)=0;
theta1(251)=0;theta2(251)=0;omega1(251)=0;omega2(251)=0;
alpha1(251)=0;alpha2(251)=0;tau1(251)=0;tau2(251)=0;
thetah1(251)=0;thetah2(251)=0;omegah1(251)=0;omegah2(251)=0;
alphah1(251)=0;alphah2(251)=0;
gx1(1001)=0;gx2(1001)=0;gy1(1001)=0;gy2(1001)=0;
 
for i=1:251%相关物理量计算
    p(:,i)=p_i+s(i)*(p_f-p_i);
    c2(i)=(p(1,i)^2+p(2,i)^2-100^2-100^2)/(2*100*100);
    s2(i)=(1-c2(i)^2)^0.5;
    c1(i)=((100+100*c2(i))*p(1,i)+100*s2(i)*p(2,i))/(p(1,i)^2+p(2,i)^2);
    s1(i)=((100+100*c2(i))*p(2,i)-100*s2(i)*p(1,i))/(p(1,i)^2+p(2,i)^2);
    thetah1(i)=atan2(s1(i),c1(i));
    thetah2(i)=atan2(s2(i),c2(i));
    theta1(i)=thetah1(i)*180/pi;
    theta2(i)=thetah2(i)*180/pi;
    
end
for i=1:250
    omega1(i)=(theta1(i+1)-theta1(i))/dt;
    omega2(i)=(theta2(i+1)-theta2(i))/dt;
    omegah1(i)=omega1(i)*pi/180;
    omegah2(i)=omega2(i)*pi/180;
end
omega1(251)=omega1(250);
omega2(251)=omega2(250);
for i=1:250
    alpha1(i)=(omega1(i+1)-omega1(i))/dt;
    alpha2(i)=(omega2(i+1)-omega2(i))/dt;
    alphah1(i)=alpha1(i)*pi/180;
    alphah2(i)=alpha2(i)*pi/180;
end
for i=1:251
    tau1(i)=m*(l/1000)^2*((3+2*cos(thetah2(i)))*alphah1(i)+(1+cos(thetah2(i)))*alphah2(i)-sin(thetah2(i))*(2*omegah1(i)+omegah2(i))*omegah2(i))+m*l/1000*9.8*(2*cos(thetah1(i))+cos(thetah1(i)+thetah2(i)));
    tau2(i)=m*(l/1000)^2*((1+cos(thetah2(i)))*alphah1(i)+alphah2(i)-sin(thetah2(i))*omegah1(i)^2)+m*l/1000*9.8*cos(thetah1(i)+thetah2(i));
end
 
%% 位置曲线
figure(1);
subplot(2,2,1);
plot(t,theta1,t,theta2);
legend('Joint1','Joint2');
xlabel('\itt');
ylabel('\theta');
title('Angle Curves');
xlim([0,5]);
 
% 速度曲线
subplot(2,2,2);
plot(t,omega1,t,omega2);
legend('Joint1','Joint2');
xlabel('\itt');
ylabel('\omega');
title('Speed Curves');
xlim([0,5]);
 
% 加速度曲线
subplot(2,2,3);
plot(t,alpha1,t,alpha2);
legend('Joint1','Joint2');
xlabel('\itt');
ylabel('\alpha');
title('Acceleration Curves');
xlim([0,5]);
 
% 力矩曲线
subplot(2,2,4);
plot(t,tau1,t,tau2);
legend('Joint1','Joint2');
xlabel('\itt');
ylabel('\tau');
title('Torque Curves');
xlim([0,5]);
 
% 末端位置曲线
figure(2);
plot(t,p(1,:),t,p(2,:));
legend('End-Effector x','End-Effector y');
xlabel('\itt');
ylabel('x/y');
title('Position of End-Effector Curves');
xlim([0,5]);
%% 导出名为‘rb8’的excel插值表
% xlswrite('rb8.xlsx',n',3,'A1');
% xlswrite('rb8.xlsx',t',3,'B1');
% xlswrite('rb8.xlsx',theta1',3,'C1');
% xlswrite('rb8.xlsx',theta2',3,'D1');
% xlswrite('rb8.xlsx',omega1',3,'E1');
% xlswrite('rb8.xlsx',omega2',3,'F1');
% xlswrite('rb8.xlsx',alpha1',3,'G1');
% xlswrite('rb8.xlsx',alpha2',3,'H1');
% xlswrite('rb8.xlsx',tau1',3,'I1');
% xlswrite('rb8.xlsx',tau2',3,'J1');
% xlswrite('rb8.xlsx',p(1,:)',3,'K1');
% xlswrite('rb8.xlsx',p(2,:)',3,'L1');
%% 绘制轨迹
figure(3);
x=0:0.1:100;
for i=1:251
   % subplot(2,2,4);
%    figure(3);
    for xn=1:length(x)
        gx1(xn)=x(xn)*cos(thetah1(i));
        gy1(xn)=x(xn)*sin(thetah1(i));
        gx2(xn)=l*cos(thetah1(i))+x(xn)*cos(thetah1(i)+thetah2(i));
        gy2(xn)=l*sin(thetah1(i))+x(xn)*sin(thetah1(i)+thetah2(i));
    end
    plot(gx1,gy1,gx2,gy2,p(1,1:i),p(2,1:i),'linewidth',1);
    legend('Link1','Link2','End-Effector Trajectory');
    xlabel('x');
    ylabel('y');
    grid on;
    axis([-200,200,-200,200]);
    title(['Time t=',num2str((i-1)/50),'s']);
end
%%
x=0:0.1:100;
i=189;
for xn=1:length(x)
        gx1(xn)=x(xn)*cos(thetah1(i));
        gy1(xn)=x(xn)*sin(thetah1(i));
        gx2(xn)=l*cos(thetah1(i))+x(xn)*cos(thetah1(i)+thetah2(i));
        gy2(xn)=l*sin(thetah1(i))+x(xn)*sin(thetah1(i)+thetah2(i));
end
plot(gx1,gy1,gx2,gy2,p(1,1:i),p(2,1:i),'linewidth',1);
legend('Link1','Link2','End-Effector Trajectory');
xlabel('x');
ylabel('y');
grid on;
axis([-200,200,-200,200]);
title(['Time t=',num2str((i-1)/50),'s']);

%% 绘制动画
M=moviein(4);
x=0:0.1:100;
for i=1:251
 %   subplot(2,2,4);
%    figure(3);
    for xn=1:length(x)
        gx1(xn)=x(xn)*cos(thetah1(i));
        gy1(xn)=x(xn)*sin(thetah1(i));
        gx2(xn)=l*cos(thetah1(i))+x(xn)*cos(thetah1(i)+thetah2(i));
        gy2(xn)=l*sin(thetah1(i))+x(xn)*sin(thetah1(i)+thetah2(i));
    end
    plot(gx1,gy1,gx2,gy2,p(1,1:i),p(2,1:i),'linewidth',1);
    legend('Link1','Link2','End-Effector Trajectory','location','SouthWest');
    xlabel('x');
    ylabel('y');
    grid on;
    axis([-200,200,-200,200]);
    title(['Time t=',num2str((i-1)/50),'s']);
    hold on
    plot(p(1,1:i),p(2,1:i),'r.','markersize',20);
% title('末端位置动画演示')
xlabel('x'),ylabel('y')
M(i)=getframe;
end
movie(M,1,200);
%%
writerObj =VideoWriter('movie_fy3.avi');





