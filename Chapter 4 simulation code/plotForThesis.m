figure
plot(t,tau(1,:),t,tau(2,:));
title(strcat('Torque on wheels Using Inv Dynamics: \Delta t=',num2str(del_t)));
xlabel('time (sec)'); ylabel('Torque (Nm)'); grid
legend ('wheel 1', 'wheel 2')

%Plot of wheel role angle with no slip dynamcis
figure ;
plot(t,theta1,t,theta2)
title('Wheel role (rad)'); xlabel('time (sec)'); ylabel('angle (rad)');grid
legend ('wheel 1', 'wheel 2')