
function [tOut,yOut,T3]= RaRs(tau,t_tau)
% This is forward dynamic simulation if RARS with slip
file1=mupad('mupadRaRs.mn');
evaluateMuPADNotebook(file1);
fT1=matlabFunction(getVar(file1,'T1')) ;
fT2=matlabFunction(getVar(file1,'T2'));
fT3=matlabFunction(getVar(file1,'T3'));
% fdT1=matlabFunction(getVar(file1,'dT1'))
% fdT2=matlabFunction(getVar(file1,'dT2'))
% fdT3=matlabFunction(getVar(file1,'dT3'))
% fomega1=matlabFunction(getVar(file1,'omega1'))
% fomega2=matlabFunction(getVar(file1,'omega2'))
% fomega3=matlabFunction(getVar(file1,'omega3'))
fdo4=matlabFunction(getVar(file1,'do4'));
fdo5=matlabFunction(getVar(file1,'do5'));
% %--------------------------------------------------
Ip=[2.89 0.84 0.21; 0.84 1.18 0.39;0.21 0.39 3.20];% Unit Kg*m^2
mp=27;% In Kg
Iw=diag([ 463.95 241.88 241.88]*1e-6);% Unit Kg*m^2
mw=.5; % In Kg
l=.210;% Span of the vehicle
r=.05; % Wheel radius
a=.220;% Location of cg of platform from rear axis
b=0.295; % Location of the line joining steering wheel pivot form platform CG


% Calculation of inertia matrices of the three bodies
% Since they are constant w.r.t independent variables they are 
% calculated outside the loop

M1=[Iw zeros(3);zeros(3) mw*eye(3)];
M2=M1;
M3=[Ip zeros(3);zeros(3) mp*eye(3)];
T1=fT1(l);T2=fT2(l);T3=fT3(a,l);
I=T1'*M1*T1+T2'*M2*T2+T3'*M3*T3;
I_Inv=inv(I);
T=[T1;T2;T3];
%------------------------------------------------------
% Torque Caclulation

function F=GenForce(ti,y,phi)
   %the integration variablt y= (th1, th2, y1, y2,  x, dth1, dth2, dy1, dy2, dx, dth4, dth5,beta,X,Y) 
    th1=y(1); th2=y(2); y1=y(3); y2=y(4);  x=y(5);
    dth1=y(6); dth2=y(7); dy1=y(8); dy2=y(9); dx=y(10); dth4=y(11); dth5=y(12);
    
    u1=interp1(t_tau,tau(1,:),ti);
    u2=interp1(t_tau,tau(2,:),ti);
    loadOnRearAxel=mp*b/(a+b);
    loadOnFrontAxel=mp*b/(a+b);
    L_r=loadOnRearAxel/2+mw;
    L_f=loadOnFrontAxel/2+mw;
    F1s=F_Trac(dy1,dth1,L_r,r);
    F1k=F_Skid(dy1,x,L_r,r);
    F2s=F_Trac(dy2,dth2,L_r,r); 
    F2k=F_Skid(dy1,x,L_r,r);
    wrench1=[u1,0,0,F1s,F1k,0];
    wrench2=[u2,0,0, F2s,F2k,0];
        
    R4=[cos(phi(1)), -sin(phi(1)),0;
        sin(phi(1)), cos(phi(1)),0;
        0,0,1];
    R5=[cos(phi(2)), -sin(phi(2)),0;
        sin(phi(2)), cos(phi(2)),0;
        0,0,1];
    do4=fdo4(a,b,dx,dy1,dy2,l);
    do5=fdo5(a,b,dx,dy1,dy2,l);
    do4_local=R4'*do4;
    do5_local=R5'*do5;
    F4s_local=F_Trac(do4_local(2),dth4,L_f,r);
    F4k_local=F_Skid(do4_local(2),do4_local(1),L_f,r);
    F5s_local=F_Trac(do5_local(2),dth5,L_f,r);
    F5k_local=F_Skid(do5_local(2),do5_local(1),L_f,r);
    F4=R4*[F4k_local,F4s_local,0]';
    F5=R5*[F5k_local,F5s_local,0]';
    F3=F4+F5;
    o34=[-l/2,b,0]; o35=[l/2,b,0];
    moment4=cross(F4,o34);    moment5=cross(F5,o35);
    moment3=moment4+moment5;
    wrench3=[moment3,F3'];
    F=[wrench1,wrench2,wrench3,F4s_local,F5s_local]
end %GenForce
%-------------------------------------------------------
function dydt=odefun(ti,y)
    %the integration variablt y= (th1,th2,y1,y2,x,dth1,dth2,dy1,dy2,dx, dth4, dth5,beta,X,Y)
    phi=[.2,.03];
    T_temp=GenForce(ti,y,phi);
    wrench=T_temp(1:18)';
   % R3=[cos(y(13)), -sin(y(13));
   %    sin(y(13)), cos(y(13))];
    dydt=zeros(10,1);
    dydt(1)=y(6);
    dydt(2)=y(7);
    dydt(3)=y(8);
    dydt(4)=y(9);
    dydt(5)=y(10);
    tempdydt=I_Inv*(T'*wrench);
    dydt(6)=tempdydt(1);
    dydt(7)=tempdydt(2);
    dydt(8)=tempdydt(3);
    dydt(9)=tempdydt(4);
    dydt(10)=tempdydt(5);
    dydt(11)=T_temp(19)/Iw(1,1);
    dydt(12)=T_temp(20)/Iw(1,1);
%     dydt(13)=T3(3,:)*y(1:5); % beta
%     dtempXY=R3*T3(4:5,:)*y(6:10)
%     dydt(14)=dtempXY(1); %X
%     dydt(15)=dtempXY(2);%Y
    

end %odef

%the integration variablt y= (th1,th2,y1,y2,x,dth1,dth2,dy1,dy2,dx,dth4, dth5)
tspan=0:max(t_tau);
ic=zeros(1,12);
[tOut,yOut] = ode23(@odefun, tspan, ic);
disp ('Completed');
%-------------------------------
% Post processing

end %RaRs