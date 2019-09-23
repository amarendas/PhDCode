function F=F_Trac(y2,y4,m,r)
lam=slip(y2,y4,m,r)
%F=.5*m*9.8;%
mu=Frectioncoff(lam);
F=mu*m*9.8;
TractiveForce=F
