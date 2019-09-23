clear;clc

%Implementation of Purepersuit for moving on a circle
% ---A.P.Das Dated 24Feb 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R= 5; % Radius of the circular path 5m
L=.5; % Look ahear dist ance 0.5m
V=.5; % Linear forward velocity of robot 0.5m/sec
b=0.5; % Seperation between the rear wheels 0.5m
%% Initial position of the robot
xr0=4.80; yr0=0; th=0;
ph0=fzero(@(ph) phi(ph,xr0,yr0,R,L),th+.2)
ph1=fzero(@(ph) phi(ph,xr0,yr0,R,L),ph0-pi/2)
xp=R*cos(ph0);yp=R*sin(ph0);% The goal point coordinates in workd coocrdinate
xp1=R*cos(ph1);yp1=R*sin(ph1);% The goal point coordinates in workd coocrdinate
Q=[cos(th) sin(th);-sin(th) cos(th)];% Rotation matrix from global to robot cordinata sys
P_r=Q*([xp;yp]-[xr0;yr0]);% Goal point in robot cordinate sys
x=P_r(1);
turning_radius=L^2/(2*x);
V_R=(V/turning_radius)*(turning_radius-b/2);
V_L=(V/turning_radius)*(turning_radius+b/2);


