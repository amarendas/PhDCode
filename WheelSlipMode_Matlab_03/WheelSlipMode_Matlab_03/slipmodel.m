function dy= slipmodel(t,y)
%y1= position of center of wheel (x) in some global cordinate 
%y2= linear velocity of the center of the wheel
%y3= angular rotation of the wheel
%y4 = angular velocity (omega) of the wheel
r=1; % radisus of wheel in m
m=2; %weight of wheel in Kg
Jw=0.5*m*r^2;% moment of inertia asuming wheel as disk
F=F_Trac(y(2),y(4),m,r);
dy(1)=y(2);
dy(2)=F/m;
dy(3)=y(4);
dy(4)=-r*F/Jw+Torque(t)/Jw;
dy=dy';