%LSPB trajectory of a single-link robot with a rotary joint

theta_0 = -5; % starting joint angle
dtheta_0 = 0; % starting joint velocity
theta_f = 80; % final joint angle
dtheta_f = 0; % final joint velocity
t_i = 0; % initial time
t_f = 4; % final time

ddtheta = 85; % acceleration constraint
t_b = (t_f/2) - (sqrt((ddtheta^2*t_f^2)-4*ddtheta*(theta_f - theta_0))/(2*ddtheta)); % blend time

v = ddtheta * t_b; % velocity of linear portion
t = t_i:0.1:t_f;

for i = 1:length(t)
    if t(i) < (t_i+t_b) % parabolic segement 1
        theta(i) = theta_0 + 0.5*(v*t(i)^2/t_b); %position
        dtheta(i) = dtheta_0 + (v*t(i)/t_b); % velocity
        ddtheta(i) = (v/t_b); % acceleration
    elseif (t(i) >= (t_i+t_b)) && (t(i) <= (t_f-t_b)) % linear sgement
        theta(i) = v*t(i)+ 0.5*(theta_f + theta_0 - v*t_f); %position
        dtheta(i) = v; % velocity
        ddtheta(i) = 0; % acceleration
    else % parabolic segement 1
        theta(i) = theta_f - 0.5*(v*t_f^2/t_b)+(v*t_f/t_b)*t(i)-0.5*(v/t_b)*t(i)^2; %position
        dtheta(i) = (v*t_f/t_b)-(v/t_b)*t(i); % velocity
        ddtheta(i) = -(v/t_b); % acceleration
    end
end

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