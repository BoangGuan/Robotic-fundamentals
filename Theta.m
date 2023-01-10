% theta(t)

function y=Theta(t)
J=259.2;
tm=0.5625;
am=86.4;
if t>=0 && t<=1/3
    y=J*t^3/6;
elseif t>1/3 && t<=2/3
    y=Theta(1/3)+Omega(1/3)*(t-1/3)+0.5*am*(t-1/3)^2;
elseif t>2/3 && t<=1
    y=Theta(2/3)+Omega(2/3)*(t-2/3)+0.5*am*(t-2/3)^2-J*(t-2/3)^3/6;
elseif t>1 && t<=1+tm
    y=Theta(1)+Omega(1)*(t-1);
elseif t>1+tm && t<=4/3+tm
    y=Theta(1+tm)+Omega(1+tm)*(t-1-tm)-J*(t-1-tm)^3/6;
elseif t>4/3+tm && t<=5/3+tm
    y=Theta(4/3+tm)+Omega(4/3+tm)*(t-4/3-tm)-0.5*am*(t-4/3-tm)^2;
else
    y=Theta(5/3+tm)+Omega(5/3+tm)*(t-5/3-tm)-0.5*am*(t-5/3-tm)^2+J*(t-5/3-tm)^3/6;
end
end
