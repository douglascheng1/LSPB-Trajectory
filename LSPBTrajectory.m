% LSPB Trajectory
clear; 
close all; 

t = [0, 2, 4]; % Time
p = [ 0, -8, -90; %position
	0, 45, 90;
	0, 0.2, 0.4];

tb = 0.5; % Given Time Blend

for i=1:3 % iterate through joints
    % Initialize/Reset Vectors
    TotalPosition = [];
    TotalVelocity = [];
    TotalAcceleration = [];
    TotalTime = [];
    
    for j =1:2 % iterate through time intervals
        %loop = j
        ti = t(j+1)-t(j); % time interval duration
        ac = 4*(p(i,j+1) - p(i,j))/ti^2; % Acceleration Constraint
        pb = p(i,j) + 0.5*ac*tb^2; % Joint Angle at Tb
        
        % parabolic segement 1
        ts1 = t(j) : tb/100 : t(j)+tb; % time segment 1 sampled
        TotalTime = [TotalTime, ts1]; %cumulative time
        position = p(i,j) + 0.5*ac*(ts1-t(j)).^2; %calculate position
        velocity = ac*(ts1-t(j)); %calculate velocity
        acceleration = ac*ones(1,101); %calculate acceleration
        TotalPosition = [TotalPosition, position]; % cumulative position
        TotalVelocity = [TotalVelocity, velocity]; % cumulative velocity
        TotalAcceleration = [TotalAcceleration, acceleration]; % cumulative accerlation
        
        % linear segment
        ts2 = t(j)+tb : ((ti-tb)-tb)/100 : (t(j+1)-tb); % time segment 2 sampled
        TotalTime = [TotalTime, ts2]; %cumulative time
        position = pb + ac*tb*1.5*((ts2-t(j))-tb); %calculate position
        velocity = ac*tb*ones(1,101); %calculate velocity
        acceleration = zeros(1,101);  %calculate acceleration
        TotalPosition = [TotalPosition, position]; % cumulative position
        TotalVelocity = [TotalVelocity, velocity]; % cumulative velocity
        TotalAcceleration = [TotalAcceleration, acceleration]; % cumulative accerlation
        
        %parabolic segement 2
        ts3 = (t(j+1)-tb) : tb/100 : t(j+1); % time segment 3 sampled
        TotalTime = [TotalTime, ts3]; %cumulative time
        position = p(i,j+1) - 0.5*ac*(ts3-t(j+1)).^2; %calculate position
        velocity = -ac*(ts3-t(j+1)); %calculate velocity
        acceleration = -ac*ones(1,101); %calculate acceleration
        TotalPosition = [TotalPosition, position]; % cumulative position
        TotalVelocity = [TotalVelocity, velocity]; % cumulative velocity
        TotalAcceleration = [TotalAcceleration, acceleration]; % cumulative accerlation
    end
    
    % Position, Velocity, Acceleration Plots
    figure
    subplot(311);
    plot(TotalTime, TotalPosition); %plot position
    title(sprintf('Position of Joint%d ',i));
    xlabel('Time (s)');
    ylabel('Position');
    subplot(312)
    plot(TotalTime, TotalVelocity); %plot velocity
    title(sprintf('Velocity of Joint%d',i));
    xlabel('Time (s)');
    ylabel('Velocity');
    subplot(313)
    plot(TotalTime, TotalAcceleration); %plot acceleration
    title(sprintf('Acceleration of Joint%d ',i));
    xlabel('Time (s)');
    ylabel('Acceleration');
end
