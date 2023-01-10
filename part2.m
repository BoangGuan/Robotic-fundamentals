% Part 2
clear;clc;
s=170; L=130; r=130; R=290;
%% draw the basic triangle frame
Bx1=0; By1=0; B1=[Bx1,By1];
Bx2=R*sqrt(3); By2=0; B2=[Bx2,By2];
Bx3=R*sqrt(3)/2; By3=1.5*R; B3=[Bx3,By3];

Bxx=[Bx1;Bx2;Bx3;Bx1];
Byy=[By1;By2;By3;By1];

plot(Bxx,Byy,'Linewidth',2)
xlabel('x(m)');ylabel('y(m)');
xlim([-100 550]);ylim([-100 550]);
%% draw the platform
hold on
xc=128; yc=500; a=pi/6;
phi1=a+pi/6; phi2=a+5/6*pi; phi3=a+3/2*pi;

Px1=xc-r*cos(phi1); Py1=yc-r*sin(phi1);
Px2=xc-r*cos(phi2); Py2=yc-r*sin(phi2);
Px3=xc-r*cos(phi3); Py3=yc-r*sin(phi3);

Pxx=[Px1;Px2;Px3;Px1];
Pyy=[Py1;Py2;Py3;Py1];

plot(Pxx,Pyy,'k','Linewidth',2)
%% Inverse Kinematics--calculate theta_i
c1=atan2(Py1,Px1);
d1=acos((s^2-L^2+Px1^2+Py1^2)/2/s/sqrt(Px1^2+Py1^2));
theta1=c1-d1;

c2=atan2(Py2,Px2-R*sqrt(3));
d2=acos((s^2-L^2+(Px2-R*sqrt(3))^2+Py2^2)/2/s/sqrt(Py2^2+(Px2-R*sqrt(3))^2));
theta2=c2-d2;

c3=atan2(Py3-1.5*R,Px3-R*sqrt(3)/2);
d3=acos((s^2-L^2+(Px3-R*sqrt(3)/2)^2+(Py3-1.5*R)^2)/2/s/((Px3-R*sqrt(3)/2)^2+(Py3-1.5*R)^2));
theta3=c3-d3;
%% Calculate M_i
Mx1=s*cos(theta1); My1=s*sin(theta1);
Mx2=R*sqrt(3)+s*cos(theta2); My2=s*sin(theta2);
Mx3=R*sqrt(3)/2+s*cos(theta3); My3=1.5*R+s*sin(theta3);
%% Plot the whole figure
vec_x1=[Bx1;Mx1;Px1]; vec_y1=[By1;My1;Py1];
vec_x2=[Bx2;Mx2;Px2]; vec_y2=[By2;My2;Py2];
vec_x3=[Bx3;Mx3;Px3]; vec_y3=[By3;My3;Py3];

hold on
plot(vec_x1,vec_y1,'ro-','Linewidth',2)
hold on
plot(vec_x2,vec_y2,'go-','Linewidth',2)
hold on
plot(vec_x3,vec_y3,'yo-','Linewidth',2)
%%
text(320,520,'(xc,yc,a)=(320,190,-pi/4)')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Workspace
clear;clc;
s=170; L=130; r=130; R=290;

Bx1=0; By1=0; B1=[Bx1,By1];
Bx2=R*sqrt(3); By2=0; B2=[Bx2,By2];
Bx3=R*sqrt(3)/2; By3=1.5*R; B3=[Bx3,By3];

Bxx=[Bx1;Bx2;Bx3;Bx1];
Byy=[By1;By2;By3;By1];

plot(Bxx,Byy,'Linewidth',2)
xlabel('x(m)');ylabel('y(m)');
xlim([-100 550]);ylim([-100 550]);

a=pi/3;
phi1=a+pi/6; phi2=a+5/6*pi; phi3=a+3/2*pi;

hold on
for xc=0:2:504
    for yc=0:2:436
        
        Px1=xc-r*cos(phi1); Py1=yc-r*sin(phi1); P1=[Px1,Py1];
        Px2=xc-r*cos(phi2); Py2=yc-r*sin(phi2); P2=[Px2,Py2];
        Px3=xc-r*cos(phi3); Py3=yc-r*sin(phi3); P3=[Px3,Py3];

        c1=atan2(Py1,Px1);
        d1=acos((s^2-L^2+Px1^2+Py1^2)/2/s/sqrt(Px1^2+Py1^2));
        theta1=c1-d1;

        c2=atan2(Py2,Px2-R*sqrt(3));
        d2=acos((s^2-L^2+(Px2-R*sqrt(3))^2+Py2^2)/2/s/sqrt(Py2^2+(Px2-R*sqrt(3))^2));
        theta2=c2-d2;

        c3=atan2(Py3-1.5*R,Px3-R*sqrt(3)/2);
        d3=acos((s^2-L^2+(Px3-R*sqrt(3)/2)^2+(Py3-1.5*R)^2)/2/s/((Px3-R*sqrt(3)/2)^2+(Py3-1.5*R)^2));
        theta3=c3-d3;
        
        d1=norm(P1-B1);
        d2=norm(P2-B2);
        d3=norm(P3-B3);

%         if (imag(theta1)==0 && imag(theta2)==0 && imag(theta3)==0)
%             Cx=(Px1+Px2+Px3)/3;
%             Cy=(Py1+Py2+Py3)/3;
%             plot(Cx,Cy,'mo')
%             hold on
%         end
        if d1<s+L&&d1>s-L&&d2<s+L&&d2>s-L&&d3<s+L&&d3>s-L
            plot(xc,yc,'m.')
            hold on
        end
    end
end
%%
text(450,520,'a=pi/3')





































