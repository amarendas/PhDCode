function T= Torque(t)
if t<=5
    T=00;
elseif ((t>5)&&(t<=30))
    T=10;
% elseif ((t>30)&&(t<=40))
%     T=-10;
 else
    T=0;
end

% if((t>=0)&&(t<=2.5))
%     T=10*t/2.5;
% end
% if ((t>2.5)&&(t<7.5))
%     T=10;
% end
% if ((t>=7.5)&&(t<=10))
%     T=10-(10/2.5)*(t-7.5);
% end
% if (t>10)
%     T=0;
% end
