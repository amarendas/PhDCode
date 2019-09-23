function F=F_Skid(linVel,lateralVel,m,r)
lam=atan2(abs(lateralVel),abs(linVel));
if lam>1
    lam=1;
end
mu=Frectioncoff(lam);
if lateralVel==0
    F=0;
elseif lateralVel<0
  F = mu*m*9.8; % Friction Forc act in opposit direction to motion
else
  F =- mu*m*9.8; % Friction Forc act in opposit direction to motion 
end