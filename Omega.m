% Omega(t)

function y=Omega(t)
J=259.2;
tm=0.5625;
am=86.4;
if t>=0 && t<=1/3
    y=0.5*J*t^2;
elseif t>1/3 && t<=2/3
    y=Omega(1/3)+am*(t-1/3);
elseif t>2/3 && t<=1
    y=Omega(2/3)+am*(t-2/3)-0.5*J*(t-2/3)^2;
elseif t>1 && t<=1+tm
    y=57.6;
elseif t>1+tm && t<=4/3+tm
    y=Omega(1+tm)-0.5*J*(t-1-tm)^2;
elseif t>4/3+tm && t<=5/3+tm
    y=Omega(4/3+tm)-am*(t-4/3-tm);
else
    y=Omega(5/3+tm)-am*(t-5/3-tm)+0.5*J*(t-5/3-tm)^2;
end
end
