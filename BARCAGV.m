%%
%BARC AGV
clear all
close all
clc
l=.4;
r=.05;
a=.101;
b=0.202;
d=.025;
h=.01;
R=5;
I1=diag([.0025 .00125 .00125]);
I2=I1;
m1=2;m2=m1;
% Inertia Paremeters of Platform
Ip=diag([0.7083 0.7083 0.4083]);
mp=25;

del_t=1e-2;
t=0:del_t:60;
%%
% DEfining the path
X=0:100:500;
Y=4*sin(2*pi*X/500);
plot(X,Y)
beta=(4*2*pi/500)*cos(2*pi*X/500);
u=beta./beta;v=beta
quiver(X,Y,u,v,)
