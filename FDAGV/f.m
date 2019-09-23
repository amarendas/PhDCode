clear all
close all
clc
l=.4;
r=.05;
a=.101;
b=0.202;
d=.025;
h=.1;
R=5;
I1=diag([.0025 .00125 .00125]);
I2=I1;
m1=2;m2=m1;
% Inertia Paremeters of Platform
Ip=diag([0.7083 0.7083 0.4083]);
mp=20;

del_t=.01;
t=0:del_t:60;

%% Calculation begins
disp('Calculation Started')
Tp=r/l*[1 -1;-a a;-l/2 -l/2];
TpI=(Tp'*Tp)\Tp';
w=2;
dx=0;
dy=R*w;
tp=[w' dx' dy']';
d_theta=TpI*tp;
%%
wnew=Tp(1,:)*d_theta;