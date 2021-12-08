%CPS trajectory of a single-link robot with a rotary joint

theta_0 = -5; % starting joint angle
theta_f= 80; % final joint angle
t_f = 4; % final time

a_0 = theta_0; %a0 coefficient
a_1 = 0; %a1 coefficient
a_2 = (3*(theta_f - theta_0) - t_f*(2*0+0))/t_f^2; %a2 coefficient
a_3 = (2*(theta_0 - theta_f) + t_f*(0+0))/t_f^3; %a3 coefficient

t = 0:0.1:t_f;

theta = a_0 + a_1*t + a_2*t.^2 + a_3*t.^3; %poosition function
dtheta = a_1 + 2*a_2*t + 3*a_3*t.^2; %velocity function
ddtheta = 2*a_2 + 6*a_3*t; %acceleration function

%plot position
figure
plot(t, theta)
title('Position')
grid on
%plot Velocity
figure
plot(t, dtheta)
title('Velocity')
grid on
%plot Acceleration
figure
plot(t, ddtheta)
title('Acceleration')
grid on