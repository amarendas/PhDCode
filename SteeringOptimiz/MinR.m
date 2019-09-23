function R_min=MinR(y)
 h=y(1);
 w=y(2);
 a=y(3);
 x=y(4);

g1=@(x)atan2(x*4*w^2,4*w^2*h+a^2*h-2*a*w*x);
R_min=w./tan(g1(x))+a/2;