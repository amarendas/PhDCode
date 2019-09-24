clear 
close all
%M=csvread('Testdata_24_09_2018_1.txt')
%M=csvread('Testdata_24_09_2018_2.txt')
%M=csvread('Test_25-09-2018.txt')
M=csvread('Test_26-09-2018_1_Line.txt')
%M=csvread('Test_26-09-2018_2_Line.txt')
%M=csvread('Test_26-09-2018_3_Completecircle.txt')
%M=csvread('Test_26-09-2018_4_half_circle.txt')
%M=csvread('CleanData_26-09-2018_1_Line.txt')

%columns of above text file
% steer Angle | Left Wheel(rpm) |Right Wheel (rpm)| Left Motor current(mA)|
% .............Right Motor cuttent(mA) |time (micrSec)
[rN,cN]=size(M);
sN=500;
G_Ratio=26; % Traction Motor gear box
L_rpm=-M(:,2)/G_Ratio; R_rpm=M(:,3)/G_Ratio;
plot (L_rpm-R_rpm)
L_mA=M(:,4); R_mA=M(:,5);
t1=M(:,6);
clear M;
r=50 % radius in mm
b=168+168 % wheel Base
Kt= 38.5/1000; % mNm/mA Torque constant of traction motor


N=rN-1; % no of data Points
del_t=zeros(N,1);
v=zeros(N,1);
omega=zeros(N,1);
x=zeros(N,1); % Xposition
y=zeros(N,1); %Y position
th=zeros(N,1); % th orientation to x axis
t_step=zeros(N,1);

 
for i=1:N-1
    if t1(i+1)>t1(i);
        del_t(i)=t1(i+1) -t1(i);
    else
        del_t(i)=1E6-t1(i)+t1(i+1);
    end
    del_t(i)=del_t(i)/1E6; % convert to sec
    
    v(i)=(1/2)*(R_rpm(i) + L_rpm(i))*(2*pi*r)/60;% mm per sec
    omega(i)=((R_rpm(i) - L_rpm (i))*2*pi)*r/b/60;% rad per sec
    
    x(i+1)=x(i)+del_t(i)*cos(th(i))*v(i);
    y(i+1)=y(i)+del_t(i)*sin(th(i))*v(i);
    th(i+1)=th(i)+del_t(i)*omega(i);
    t_step(i+1)=t_step(i)+del_t(i);
   i
   v(i)
   omega(i)
    x(i+1)
    y(i+1)
    th(i+1)
    
end
plotN=N;
plot(x(1:plotN),y(1:plotN));title ('Path of Robot (mm)');xlabel('x (mm)'); ylabel('y (mm)'); axis([0 5000 -200 200]) ; %daspect([1 1 1])
figure;plot(t_step,v(1:plotN));title('Linear velocity of Mobile Robot');xlabel('time (sec)'); ylabel('mm/sec')
figure;plot(t_step,omega(1:plotN));title('Angular Velocity of Robot \omega_3');xlabel('time (sec)'); ylabel('rad/sec')
% Current to torque conversion

L_Tm=-Kt*L_mA(1:N)*G_Ratio; %in mNm
R_Tm=Kt*R_mA(1:N)*G_Ratio; %in mNm
Ft=(L_Tm+R_Tm)/(r*10^(-3)); %in mN
Tq=((L_Tm-R_Tm)/(r*10^(-3)))*b/2*10^-3; %in mN
% figure; plot(1:N,L_mA, 1:N, R_mA); 
 figure; plot(t_step,L_Tm, t_step,R_Tm);title 'torque of rear motors';xlabel('time (sec)'); ylabel('mN-m'); legend('Left Motor','Right Motot');
 %figure; plot(Ft);title 'Force';

%disp('All values in msec')
% t_mean=mean(dif(10:N-5))/1000
% t_std=std(dif(10:N-5))/1000
% t_max=max(dif(10:N-5))/1000
% t_min=min(dif(10:N-5))/1000

