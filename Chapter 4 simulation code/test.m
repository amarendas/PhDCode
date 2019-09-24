clear all
close all
clc
l=.4;r=.05;a=.101;b=0.202;d=.025;h=.01;R=5;
del_t=1e-4;
t=0:del_t:60;

Tp=r/l*[1 -1;-a a;-l/2 -l/2];
TpI=pinv(Tp);

w=beta_dot(t); % calculating omega 'w' based on rate of change of beta
dx=0*t;
dy=R*w;
tp=[w' dx' dy']';
d_theta=TpI*tp;

temp=Tp*d_theta; % calculating omega 'w' based on theta dot calculated 
w_ulta=temp(1,:);

plot(t,w_ulta-w); title('error in omega '); xlabel('time (sec)'),ylabel('rad'); grid;