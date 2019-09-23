function db=beta_dot(t)
a2=pi/60^2;
a3=-2*pi/60^3;
a4=pi/60^4;
db=a2*t.^2+a3*t.^3+a4*t.^4;

