function dwdt=IntOmega(ti,w,Tout,Yout,T3)
twmp1=interp1(Tout,Yout(:,6:10),ti)';
twist=T3*twmp1;
Twist2D=twist(3:5);
dwdt(1)=Twist2D(1);
R=[cos(w(1)), -sin(w(1)); sin(w(1)),cos(w(1))]
vWorld=R*Twist2D(2:3);
dwdt(2)=vWorld(1);
dwdt(3)=vWorld(2);
dwdt=dwdt'
end