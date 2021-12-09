%CPS trajectory given joint positions and velocites and time

p = [0,-8,-90;0,45,90;0,0.2,0.4]; % Define Positions
v = [0,10,0;0,40,0;0,0.2,0]; % Define Velocities
t = [0, 2, 4];

totalPosition = [];
totalVelocity = [];
totalAcceleration = [];

sample = 0.1;

for i = 1:size(p)
    for j=1:(length(t)-1)
        j
        a_0 = p(i,j) %a0 coefficient
        a_1 = v(i,j); %a1 coefficient
        a_2 = (3*(p(i,j+1) - p(i,j)) - (t(j+1)-t(j))*(2*v(i,j)+v(i,j+1)))/((t(j+1)-t(j))^2);
        a_3 = ((2*(p(i,j) - p(i,j+1))) + (t(j+1)-t(j))*(v(i,j)+v(i,j+1)))/((t(j+1)-t(j))^3); %a3 coefficient

        ts = (t(j):sample:t(j+1))-t(j);

        theta = a_0 + a_1*ts + a_2*ts.^2 + a_3*ts.^3; %position function
        dtheta = a_1 + 2*a_2*ts + 3*a_3*ts.^2; %velocity function
        ddtheta = 2*a_2 + 6*a_3*ts; %acceleration function

        totalPosition = [totalPosition, theta];
        totalVelocity = [totalVelocity, dtheta];
        totalAcceleration = [totalAcceleration, ddtheta];

        if j ~= (length(t)-1)
            totalPosition(end) = [];
            totalVelocity(end) = [];
            totalAcceleration(end) = [];
        else
            time = t(1):sample:t(length(t));
            %plot position
            figure
            subplot(3,1,1);
            plot(time, totalPosition)
            title(sprintf('Position of Joint %d',i));
            grid on
            %plot Velocity
            subplot(3,1,2);
            plot(time, totalVelocity)
            title(sprintf('Velocity of Joint %d',i));
            grid on
            %plot Acceleration
            subplot(3,1,3);
            plot(time, totalAcceleration)
            title(sprintf('Acceleration of Joint %d',i));
            grid on
            
            totalPosition = [];
            totalVelocity = [];
            totalAcceleration = [];
        end
    end
end