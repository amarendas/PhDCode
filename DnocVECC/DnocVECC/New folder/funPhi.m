function dPhi=funPhi(t,y)
l=.4;
r=.05;
a=.101;
b=0.202;
d=.025;
h=.1;
R=5;
Tp=r/l*[1 -1;-a a;-l/2 -l/2];
TpI=inv(Tp'*Tp)*Tp';
w=beta_dot(t);
dx=0*t;
dy=R*w;
tp=[w' dx' dy']';
d_theta=TpI*tp;
th3=y(1);
phi=y(2);

dPhi=-[-d* cos(phi) d*sin(phi);r*sin(phi) r*cos(phi)]*[-sin(phi)/l sin(phi)/l;... 
    1/d-(a+b)/(l*d)-cos(phi)/l  (a+b)/(l*d)+cos(phi)/l ]*d_theta;