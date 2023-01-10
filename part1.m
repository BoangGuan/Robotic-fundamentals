clear;clc;

%% FK matrix
format short
syms t1 t2 t3 t4 t5
L1=0.1;L3=0.1;L4=0.1;L5=0.1;

T01(t1)=[cos(t1),-sin(t1),0,0;
         sin(t1),cos(t1), 0,0;
         0,      0,       1,L1;
         0,      0,       0,1];
T12(t2)=[cos(t2),-sin(t2),0,0;
         0,      0,      -1,0;
         sin(t2),cos(t2), 0,0;
         0,      0,       0,1];
T23(t3)=[cos(t3),-sin(t3),0,L3;
         sin(t3), cos(t3),0,0;
         0,       0,      1,0;
         0,       0,      0,1];
T34(t4)=[cos(t4),-sin(t4),0,L4;
         sin(t4), cos(t4),0,0;
         0,       0,      1,0;
         0,       0,      0,1];
T45(t5)=[cos(t5),-sin(t5),0,0;
         0,       0,      1,L5;
        -sin(t5),-cos(t5),0,0;
         0,       0,      0,1];

T05=T01(t1)*T12(t2)*T23(t3)*T34(t4)*T45(t5);

% format short
% subs(T05,[t1,t2,t3,t4,t5],[0,0,0,-90,90]*pi/180)
%% FK2DOF
q1=[60,70,80,90]'*pi/180;
q2=[-30,-35,-40,-45]'*pi/180;
q3=[45,55,60,65]'*pi/180;
q4=[-60,-55,-50,-45]'*pi/180;
q=[q1,q2,q3,q4];

c1=cos(q1);s1=sin(q1);
c2=cos(q2);s2=sin(q2);
c3=cos(q3);s3=sin(q3);c23=cos(q2+q3);s23=sin(q2+q3);
c4=cos(q4);s4=sin(q4);c234=cos(q2+q3+q4+90*pi/180);s234=sin(q2+q3+q4+90*pi/180);
%% Tip Position
xt=(L3*c2+L4*c23+L5*c234).*c1;
yt=(L3*c2+L4*c23+L5*c234).*s1;
zt=L1+L3*s2+L4*s23+L5*s234;

pt=[xt,yt,zt];
%% Plot the trajectory of the end-effector
figure(1)
set(1,'position',[680 358 560 420])

plot3(pt(1,1),pt(1,2),pt(1,3),'rx')
hold on
plot3(pt(2:4,1),pt(2:4,2),pt(2:4,3),'x')
title('Tip Trajectory') ; xlabel('x (m)') ; ylabel('y (m)') ;
%% Plot the robotic arm, in 4 different positions
figure (2) 
set(2,'position',[116 190 560 420])

x0=zeros(4,1);
y0=zeros(4,1);
z0=zeros(4,1);

x1=zeros(4,1);
y1=zeros(4,1);
z1=ones(4,1)*0.1;

x2=zeros(4,1);
y2=zeros(4,1);
z2=ones(4,1)*0.1;

x3=x2+L3*c2.*c1; % the end tip of the third stick
y3=y2+L3*c2.*s1;
z3=z2+L3*s2;

x4=x3+L4*c23.*c1;
y4=y3+L4*c23.*s1;
z4=z3+L4*s23;

for i=1:4
    xx=[x0(i);x1(i);x2(i);x3(i);x4(i);pt(i,1)];
    yy=[y0(i);y1(i);y2(i);y3(i);y4(i);pt(i,2)];
    zz=[z0(i);z1(i);z2(i);z3(i);z4(i);pt(i,3)];
    
    plot3(xx,yy,zz,'ko-','Linewidth',2)
    axis equal
    hold on
    
    text(pt(1,1),pt(1,2),pt(1,3),'x');text(pt(1,1)+0.002, pt(1,2)+0.002, pt(1,3)+0.002, 'ptStart');
    text(pt(4,1),pt(4,2),pt(4,3),'x');text(pt(4,1)+0.002, pt(4,2)+0.002, pt(4,3)+0.002, 'ptEnd');
    axis equal
    pause(0.5)
%     hold off
    pause(0.5)
end
xlabel('x(m)');ylabel('y(m)');zlabel('z(m)');    
% xx=[x0(4);x1(4);x2(4);x3(4);x4(4);pt(4,1)];
% yy=[y0(4);y1(4);y2(4);y3(4);y4(4);pt(4,2)];
% zz=[z0(4);z1(4);z2(4);z3(4);z4(4);pt(4,3)];
%% Workspace
syms t1 t2 t3 t4 t5
L1=0.1;L3=0.1;L4=0.1;L5=0.1;

q1=-90*pi/180:12*pi/180:90*pi/180-pi/180;
q2=0:12*pi/180:180*pi/180-pi/180;
q3=-135*pi/180+pi/180:12*pi/180:45*pi/180;
q4=-180*pi/180:12*pi/180:0-pi/180;

len=length(q1)*length(q2)*length(q3)*length(q4);
xt=zeros(len,1);
yt=zeros(len,1);
zt=zeros(len,1);
%%
i=1;

for a=1:15
    c1=cos(q1(a));s1=sin(q1(a));

    for b=1:15
        c2=cos(q2(b));s2=sin(q2(b));

        for c=1:15
            c3=cos(q3(c));s3=sin(q3(c));c23=cos(q2(b)+q3(c));s23=sin(q2(b)+q3(c));

            for d=1:15          
                c4=cos(q4(d));s4=sin(q4(d));c234=cos(q2(b)+q3(c)+q4(d)+90*pi/180);s234=sin(q2(b)+q3(c)+q4(d)+90*pi/180);

                xt(i,1)=(L3*c2+L4*c23+L5*c234)*c1;
                yt(i,1)=(L3*c2+L4*c23+L5*c234)*s1;
                zt(i,1)=L1+L3*s2+L3*s23+L5*s234;
                
                i=i+1;
            end
        end
    end
end


figure(1)
plot3(xt,yt,zt,'.')
xlabel('x')
ylabel('y')
zlabel('z')
title('3D workspace')

figure(2)
plot(xt,yt,'.')
xlabel('x')
ylabel('y')
title('2D workspace: x-y space')

figure(3)
plot(xt,yt,'.')
xlabel('x')
ylabel('z')
title('2D workspace: x-z space')

figure(4)
plot(xt,yt,'.')
xlabel('y')
ylabel('z')
title('2D workspace: y-z space')

axis equal
%% Inverse Kinematics
syms xg yg zg phi mu
phi=-pi/2;
mu=0;
xg=0.2;
yg=-0.1;
zg=0.1;

t1=atan2(yg,xg);
t5=mu;

rg=sqrt(xg*xg+yg*yg);
r4=rg+L5*sin(phi);
z4=zg-L5*cos(phi);

D=(L3*L3+L4*L4-r4*r4-(z4-0.1)^2)/2/L3/L4;
t3=atan2(-sqrt(1-D*D),-D);

theta=atan2((z4-L1),r4);
alpha=atan2(-L4*sin(t3),(L3+L4*cos(t3)));
t2=theta+alpha;

t4=phi-t2-t3;

%%
% figure(1)
% set(2,'position',[116 190 560 420])
% hold on

x5=xg;y5=yg;z5=zg;
p5=[x5,y5,z5];

x0=0;y0=0;z0=0;
p0=[x0,y0,z0];

x1=0;y1=0;z1=0.1;
p1=[0,0,0.1];

x2=0;y2=0;z2=0.1;
p2=[0,0,0.1];

x3=L3*cos(t2)*cos(t1);
y3=L3*cos(t2)*sin(t1);
z3=z2+L3*sin(t2);
p3=[x3,y3,z3];

x4=x3+L4*cos(t2+t3)*cos(t1);
y4=y3+L4*cos(t2+t3)*sin(t1);
z4=zg-L5*cos(phi);
p4=[x4,y4,z4];

xx=[x0;x1;x2;x3;x4;x5];
yy=[y0;y1;y2;y3;y4;y5];
zz=[z0;z1;z2;z3;z4;z5];

plot3(xx,yy,zz,'ro-','Linewidth',2)
xlim([0 0.3])
ylim([-0.2 0.2])
xticks(0:0.05:0.3)
yticks(-0.2:0.05:0.2)
zticks(0:0.05:0.2)
% axis equal
xlabel('x(m)');ylabel('y(m)');zlabel('z(m)');
grid on
%% Trajactory
%% Free Motion
% hold on
x=ones(1,100)*0.2;
y=0.1:-0.002:-0.1+0.002;
coef1=-2667;coef2=-7.628e-14;coef3=26.26;coef4=3.165e-16;coef5=0.1;
z=coef1.*(y.^4)+coef2.*(y.^3)+coef3.*(y.^2)+coef4.*y+coef5;

plot3(x,y,z,'r-')
title('Free Motion')
%% Straight Line
% hold on
x1=ones(1,20)*0.2;
y1=linspace(0.1,0.05,20);
z1=linspace(0.1,0.15,20);
plot3(x1,y1,z1,'r-')
hold on
x2=ones(1,20)*0.2;
y2=linspace(0.05,0,20);
z2=linspace(0.15,0.1,20);
plot3(x2,y2,z2,'r-')
hold on
x3=ones(1,20)*0.2;
y3=linspace(0,-0.05,20);
z3=linspace(0.1,0.15,20);
plot3(x3,y3,z3,'r-')
hold on
x4=ones(1,20)*0.2;
y4=linspace(-0.05,-0.1,20);
z4=linspace(0.15,0.1,20);
plot3(x4,y4,z4,'r-')
title('Straight Line Motion')
%% Obstacle Line
%hold on
[X,Y,Z]=sphere;
r=0.0125;
X2=X*r;
Y2=Y*r;
Z2=Z*r;
surf(X2+0.2,Y2+0.0125,Z2+0.125)
shading interp
hold on
x1=ones(1,20)*0.2;
y1=linspace(0.1,0.05,20);
z1=linspace(0.1,0.15,20);
plot3(x1,y1,z1,'r-')
hold on
x_m=ones(1,10)*0.2;
y_m=linspace(0.05,0.025,10);
z_m=linspace(0.15,0.1,10);
plot3(x_m,y_m,z_m,'r-')
hold on
x2=ones(1,20)*0.2;
y2=linspace(0.025,0,20);
z2=linspace(0.1,0.1,20);
plot3(x2,y2,z2,'r-')
hold on
x3=ones(1,20)*0.2;
y3=linspace(0,-0.05,20);
z3=linspace(0.1,0.15,20);
plot3(x3,y3,z3,'r-')
hold on
x4=ones(1,20)*0.2;
y4=linspace(-0.05,-0.1,20);
z4=linspace(0.15,0.1,20);
plot3(x4,y4,z4,'r-')
title('Obstacle Line Motion')







































