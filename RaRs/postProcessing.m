
Tspan=0:.1:60;
[Time,Omega]=ode23(@(ti,w) IntOmega(ti,w,Tout,Yout,T3),Tspan,[0,0,0]);

