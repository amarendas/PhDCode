function dq=FD(ti,q,t,tau)
d_theta=[q(3);q(4)];

l=.210;% Span of the vehicle
r=.05; % Wheel radius
a=.220;% Location of cg of platgorm from rear axis
b=0.295;
%d=.025;
%h=.01;
%Inertial parameters of the wheel
% Unit Kg*m^2
I1=diag([241.88 241.88 463.95]*1e-6);
I2=I1;
m1=.5;m2=m1; % In Kg
% Inertia Paremeters of Platform
Ip=[2.89 0.84 0.21; 0.84 1.18 0.39;0.21 0.39 3.20];
mp=27;

Tp=r/l*[1 -1;-a a;-l/2 -l/2];
w=Tp(1,:)*d_theta;
%% 
    T1=[1 0; 0 0; r/l -r/l; 0 0;-r 0; 0 0];
    dT1=[0 0;w 0; 0 0 ;r*w 0;0 0; 0 0];
    w1=T1(1:3,:)*d_theta;
    Omega1=CrossProMatrix(w1);
    W1=[Omega1 zeros(3); zeros(3, 6)];
    M1=[I1 zeros(3);zeros(3) m1*eye(3)];
    
    %% For body 2
    T2=[ 0 1; 0 0; r/l -r/l; 0 0; 0 -r; 0 0 ];
    dT2=[0 0; 0 w; 0 0; 0 w*r;0 0; 0 0];
    w2=T2(1:3,:)*d_theta;
    Omega2=CrossProMatrix(w2);
    W2=[Omega2 zeros(3); zeros(3, 6)];
    M2=[I2 zeros(3);zeros(3) m2*eye(3)];
    
    %% Platform
    T5=[0 0; 0 0 ;Tp; 0 0;];
    dT5=[0 0; 0 0; 0 0;r*w/2 r*w/2; -a*r*w/l a*r*w/l; 0 0];
    w5=T5(1:3,:)*d_theta;
    Omega5=CrossProMatrix(w5);
    W5=[Omega5 zeros(3); zeros(3, 6)];
    M5=[Ip zeros(3);zeros(3) mp*eye(3)];
    
    %% Calculating the torque on each wheels
    M=blkdiag(M1, M2,M5);
    W=blkdiag(W1,W2,W5);
    T=[T1;T2;T5];
    dT=[dT1;dT2;dT5];
%     disp(' I_FD ');
    I=T'*M*T;
   % disp ('FC C');
    C=-T'*(M*dT+W*M*T);
    P=[zeros(2,2) eye(2,2);zeros(2,2) I\C];
    Q=[zeros(2,2); I\eye(2,2)];
    u1=interp1(t,tau(1,:),ti);
    u2=interp1(t,tau(2,:),ti);
    dq=P*q+Q*[u1;u2];

  