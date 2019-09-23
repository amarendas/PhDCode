clear ; close all
T=100;
t=0:.2:T;
N=max(size(t));
Tq(N)=0;
for i=1:N
Tq(i)=Torque(t(i));
end
subplot(1,2,1)
plot(t,Tq);grid
title 'Torque applied to the Wheel'
xlabel 't (sec)'
ylabel 'Torque(Nm)'

%figure;

lm=linspace(-1,1,50);
fric(50)=0;
for i=1:50
fric(i)=Frectioncoff(lm(i));
end
subplot(1,2,2)
plot(lm,fric)
title 'Variation of Friction coeff with slip'
xlabel 'slip,\lambda'
ylabel '\mu'
grid

% 
% del_t=0.0001;
% t=0;
% y=[0,0,0,0]';
% for i= 0:100
% y=slipmodel(t,y)*del_t+y
% t=t+del_t
% disp '---------------'
% end

r=1;
m=2;
% check function "slip model"

%simulation of wheel rolling with slip
[TOUT,YOUT] = ode15s(@slipmodel,[0 T],[0;0;0;0])

%figure
s(max(size(TOUT)))=0;
for i=1:max(size(TOUT))
s(i)=slip(YOUT(i,2),YOUT(i,4),m,r);
F_f(i)=F_Trac(YOUT(i,2),YOUT(i,4),m,r);
Mu(i)=Frectioncoff(s(i));
end


figure
subplot(2,2,3)
plot(TOUT,s);
title 'Slip of wheel'
xlabel 't(sec)'
ylabel 'slip(\lambda)'
grid

subplot(2,2,1)
plot(TOUT,YOUT(:,2));
title 'velocity Translational'
xlabel 't(sec)'
ylabel 'V';grid

subplot(2,2,2)
plot(TOUT,YOUT(:,4)*r);
title 'Omega times R'
xlabel 't(sec)'
ylabel 'Peripheral velocity Vp';grid

subplot(2,2,4)
plot(TOUT,F_f);
title 'Traction Force'
xlabel 't(sec)'
ylabel 'F_f(N)';grid

% Combined plots
figure
h = plot(TOUT,YOUT(:,2),TOUT,YOUT(:,4)*r);grid minor
legend(h,'V(m/sec)','w.r(m/sec')
xlabel('Time (sec)') 
title('Multiple Decay Rates') 