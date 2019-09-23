function F=F_Trac(linVel,omega,m,r)
lam=slip(linVel,omega,m,r);
%F=.5*m*9.8;%
mu=Frectioncoff(lam);
F=mu*m*9.8;

