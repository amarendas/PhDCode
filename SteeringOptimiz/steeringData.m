% steering data
h=30;
w=500;
a=210;
%dis=a/2-b-10;
y0=[h,w,a,0];
LB=[30;515;50;0];
UB=[200;600; 270;inf];
A=[0,0,0,-1];
B=0;

options = optimoptions('fmincon',...
    'Algorithm','interior-point','Display','iter');

X=fmincon(@MinR,y0,A,B,[],[],LB,UB,@nonlincon1,options)
h=X(1)
w=X(2)
a=X(3)
dis=X(4)
b=0.5*a*h/w
R_min=MinR(X)
nonlincon1(X)