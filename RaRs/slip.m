function lam=slip(linVel,omega,m,r)
% Calculate slip

%y1= position of center of wheel (x) in some global cordinate 
%y2= linear velocity of the center of the wheel
%y3= angular rotation of the wheel
%y4 = angular velocity (omega) of the wheel
% linVelBar: is linear velocity ar wheel circumferance
% slip: it indicates the relativelocity of contatct point with ground

linVelBar=omega*r;
a_linVel=abs(linVel);
a_linVelBar=abs(linVelBar);

slip=(a_linVelBar -a_linVel);

if (slip)==0
    lam=0;
else
    if a_linVelBar >a_linVel
        lam= slip/a_linVelBar;
        else
        lam = slip/a_linVel;        
    end
end

if abs(lam)>1
    lam=lam/abs(lam);
end
%lam
