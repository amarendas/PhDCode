function lam=slip(y2,y4,m,r)
% Calculate slip

%y1= position of center of wheel (x) in some global cordinate 
%y2= linear velocity of the center of the wheel
%y3= angular rotation of the wheel
%y4 = angular velocity (omega) of the wheel
% linVelBar: is linear velocity ar wheel circumferance
% slip: it indicates the relativelocity of contatct point with ground
linVel=y2;
omega=y4;
linVelBar=omega*r;
a_linVel=abs(linVel);
a_linVelBar=abs(linVelBar);
% if omega <0 
%     disp ' omega is negative'
% end
% if linVel < 0
%     disp 'lin vel is negative'
% end
slip=(linVelBar -linVel);

if (slip)==0
    lam=0;
else
    if a_linVelBar >a_linVel
        lam= abs(slip)/linVelBar;
    else
        lam = slip/linVel;        
    end
end
if abs(lam)>1
    lam=lam/abs(lam);
end
%lam
