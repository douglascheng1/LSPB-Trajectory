%CPS trajectory given joint positions and velocites and time

p = [0,-8,-90;0,45,90;0,0.2,0.4]; % Define Positions
v = [0,10,0;0,40,0;0,0.2,0]; % Define Velocities
t = [0, 2, 4];

totalPosition = []; %intialize position array
totalVelocity = []; %intialize velocity array
totalAcceleration = []; %intialize acceleration array

sample = 0.1 % sampe time period

for i = 1:size(p) % iterate through # of joints
    for j=1:(length(t)-1) % iterate through time peroids
        j
        a_0 = p(i,j) %a0 coefficient
        a_1 = v(i,j); %a1 coefficient
        a_2 = (3*(p(i,j+1) - p(i,j)) - (t(j+1)-t(j))*(2*v(i,j)+v(i,j+1)))/((t(j+1)-t(j))^2); %a1 coefficient
        a_3 = ((2*(p(i,j) - p(i,j+1))) + (t(j+1)-t(j))*(v(i,j)+v(i,j+1)))/((t(j+1)-t(j))^3); %a3 coefficient

        ts = (t(j):sample:t(j+1))-t(j); % sample time within time peroid

        pos = a_0 + a_1*ts + a_2*ts.^2 + a_3*ts.^3; %position function
        vel = a_1 + 2*a_2*ts + 3*a_3*ts.^2; %velocity function
        acc = 2*a_2 + 6*a_3*ts; %acceleration function

        totalPosition = [totalPosition, pos]; % update position array
        totalVelocity = [totalVelocity, vel]; % update velocity array
        totalAcceleration = [totalAcceleration, acc]; % update acceleration array

        if j ~= (length(t)-1) % delete last element of trajectory array so next time peroid can start there
            totalPosition(end) = [];
            totalVelocity(end) = [];
            totalAcceleration(end) = [];
        else %plot trajectory
            time = t(1):sample:t(length(t)); % sample tiem over entire time period
            figure
            %plot position
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
            
            % reset trajectory arrays for next joint
            totalPosition = []; 
            totalVelocity = [];
            totalAcceleration = [];
        end
    end
end
