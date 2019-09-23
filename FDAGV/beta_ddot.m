function ddb=beta_ddot(t)
a1=2*pi/60^3;
a2=-6*pi/60^3;
a3=4*pi/60^4;
ddb=a1*t + a2*t.^2 + a3*t.^3;