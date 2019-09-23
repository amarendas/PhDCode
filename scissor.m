function [ ] = scissor(  )

s=60; 
A=[s^3 s^4 s^5;
3*s^2 4*s^3 5*s^4;
6*s 12*s^2 20*s^3];
b=[30*3.14/180 0 0]'
a=A\b
a3=a(1); a4=a(2); a5=a(3);
% direct dynamics
    function val=xt(t)
         val=a3*t^3+a4*t^4+a5*t^5;
    end
    function val=dxt(t)
                val=a3*3*t^2+a4*4*t^3+a5*5*t^4;
    end;
    function val=ddxt(t)
          val=a3*6*t+a4*12*t^2+a5*20*t^3;
    end
m=650e-3; % kg  -mass of link
l=300e-3; %m -length
thk=20e-3; %m - thick
wid=40e-3;%m - width
I=m*(l^+wid^2)/12; % inertia of link

t=0:.1:60;
N=max(size(t));
T=0*t; % generate place holde
ThetaGiven=0*t;
disp('Inverse Dynamics Started');
for i=1:N
    ti=t(i);
T(i)=0.045*ddxt(ti)*(4.5683*sin(xt(ti))^2+2604.848*cos(xt(ti))^2+8*I)- 0.045*dxt(ti)^2*sin(xt(ti))*cos(xt(ti))*(4.5683*sin(xt(ti))^2+2604.848*cos(xt(ti))^2)+158.197*cos(xt(ti));
ThetaGiven(i)=xt(t(i));
end
plot(t,T); title('Inverse Dynamics of scissor'); xlabel('time (t)-sec'); ylabel('Torque-N*m')

function dq=FD(ti,q,t,tau)
   Tq=interp1(t,tau,ti);
  dq1=q(2);
  dq2=Tq+ 0.045*q(2)^2*sin(q(1))*cos(q(1))*(4.5683*sin(q(1))^2+2604.848*cos(q(1))^2)- 158.197*cos(q(1));
  dq2=dq2/(0.045*(4.5683*sin(q(1))^2+2604.848*cos(q(1))^2+8*I));
  dq=[dq1;dq2];

end


%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
q0=[0 0 ]';% initial condition
disp('Forward Dynamics Started-scissor');
%options = odeset('RelTol',1e-06, 'AbsTol',1e-12);
[FD_Time,FD_q]=ode45(@(ti,y) FD(ti,y,t,T),t,q0);

disp('Forward dynamics complete')
figure
plot(FD_Time,FD_q(:,1));title('Forward Dynamics'); xlabel('time'); ylabel('theta');
calTheta=interp1(FD_Time,FD_q(:,1),t);
figure; title('ThetaGiven');plot(t,calTheta,t,ThetaGiven);xlabel('time-sec '); ylabel('theta radians')

end
