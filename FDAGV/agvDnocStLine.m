% saha 1989 IEEE
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
mp=35;

del_t=.1;
t=0:del_t:60;
%% Calculation begins
disp('Calculation Started')
Tp=r/l*[1 -1;-a a;-l/2 -l/2];
TpI=inv(Tp'*Tp)*Tp';
w=0*t;
dx=0*t;
dy=beta_dot(t);
tp=[w' dx' dy']';
d_theta=TpI*tp;
%integrate d_theta to get theta
g=tf(1,[1 0]);
theta1=lsim(g,d_theta(1,:),t);
theta2=lsim(g,d_theta(2,:),t);
% plot role angle  of each wheel
plot(t,theta1,t,theta2)
title('Wheel role (rad)'); xlabel('time (sec)'); ylabel('angle (rad)');grid
legend('wheel 1', 'Wheel 2')
%%plot role velocity of each wheel
figure
plot(t,d_theta(1,:),t,d_theta(2,:))
title('Wheel angular velocity'); xlabel('time(sec)'); ylabel('angle rate (rad/sec)');grid;
legend('wheel 1', 'Wheel 2')


% dw=0*t;
% ddx=-R*w.^2;
% ddy=R*dw;
% dtp=[dw' ddx' ddy']';
% % Calculating theta_3 and phi
% [tim1, theta_u]=ode23(@funPhi,t,[0 0]);

dd_theta=zeros(2, length(t));
d_theta_u=zeros(2, length(t));
dd_v=zeros(3, length(t));
for n=1:length(t)
    %d_theta_u(:,n)=funPhi(t(n),theta_u(n,:));
    %differentiate dtheta to get acceleration
    if n>1
        dd_theta(:,n)=(d_theta(:,n)-d_theta(:,n-1))/del_t;
    else
        dd_theta(:,n)=d_theta(:,n)/del_t;
    end
    
    %dd_v(:,n)=Tp*dd_theta(:,n)+(r/l)*w(n)*[0 0 ;l/2 l/2; -a a]*d_theta(:,n);
end
figure 
plot(t,dd_theta(1,:), t, dd_theta(2,:));
title ('Accleration of wheel'); xlabel('time(sec)'); ylabel('acce(r/s^2)');grid;

disp('Calculation of Wheel angle, angula velocity and accleration completed');


%% Inverse Dynamics
% calculation of Torqu

disp('Inverse dynamics started')
tau=zeros(2,length(t));
for n=1:length(t)
    %% For body 1
    
    T1=[1 0; 0 0; r/l -r/l; 0 0;-r 0; 0 0];
    dT1=[0 0;w(n) 0; 0 0 ;r*w(n) 0;0 0; 0 0];
    %t1=T1*d_theta(:,n)
    %dt1=T1*dd_theta(:,n)+dT1*d_theta(:,n);
    w1=T1(1:3,:)*d_theta(:,n);
    Omega1=CrossProMatrix(w1);
    W1=[Omega1 zeros(3); zeros(3, 6)];
    M1=[I1 zeros(3);zeros(3) m1*eye(3)];
    
    %% For body 2
    T2=[ 0 1; 0 0; r/l -r/l; 0 0; 0 -r; 0 0 ];
    dT2=[0 0; 0 w(n); 0 0; 0 w(n)*r;0 0; 0 0];
    w2=T2(1:3,:)*d_theta(:,n);
    Omega2=CrossProMatrix(w2);
    W2=[Omega2 zeros(3); zeros(3, 6)];
    M2=[I2 zeros(3);zeros(3) m2*eye(3)];
    
    %% Platform
    T5=[0 0; 0 0 ;Tp; 0 0;];
    dT5=[0 0; 0 0; 0 0;r*w(n)/2 r*w(n)/2; -a*r*w(n)/l a*r*w(n)/l; 0 0];
    w5=T5(1:3,:)*d_theta(:,n);
    Omega5=CrossProMatrix(w5);
    W5=[Omega5 zeros(3); zeros(3, 6)];
    M5=[Ip zeros(3);zeros(3) mp*eye(3)];
    
    %% Calculating the torque on each wheels
    M=blkdiag(M1, M2,M5);
    W=blkdiag(W1,W2,W5);
    T=[T1;T2;T5];
    dT=[dT1;dT2;dT5];
    tau(:,n)=T'*M*T*dd_theta(:,n)+T'*(M*dT+W*M*T)*d_theta(:,n);
    
end
figure;plot(t,tau(1,:),t,tau(2,:));title(strcat('Torque on wheels:-del_t=',num2str(del_t)));
xlabel('time (sec)'); ylabel('Torque (Nm)'); grid

disp('Inverse Dynamics completed')

%%
%%%calculating the forward dynamics
q0=[0 0 0 0]';% initial condition
disp('Forward Dynamics Started');

[FD_Time,FD_q]=ode45(@(ti,y) FD(ti,y,t,tau),t,q0);

disp('Forward dynamics complete')
figure 
plot(FD_Time,FD_q(:,1),FD_Time,FD_q(:,2)); grid; 
title('Wheel rol angle using Fwd dyn'),xlabel('time(sec)'),ylabel('rad');
figure
plot(FD_Time,FD_q(:,1)-theta1, FD_Time,FD_q(:,2)-theta2);grid
title('error in angle ');xlabel('time(sec)');ylabel('rad');
disp(' End of run')
