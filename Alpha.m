% Alpha(t)

function y=Alpha(t)
J=259.2;
tm=0.5625;
am=86.4;
if t>=0 && t<=1/3
    y=J*t;
elseif t>1/3 && t<=2/3
    y=am;
elseif t>2/3 && t<=1
    y=am-J*(t-2/3);
elseif t>1 && t<=1+tm
    y=0;
elseif t>1+tm && t<=4/3+tm
    y=-J*(t-1-tm);
elseif t>4/3+tm && t<=5/3+tm
    y=-am;
else
    y=-am+J*(t-5/3-tm);
end
end
