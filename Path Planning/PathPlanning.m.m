% Add
close all;
clear;

% Defining the Model Robot (link lengths may be different) 
Link1 = Link('d', 117.6, 'a', 0, 'alpha', pi/2);
Link2 = Link('d', 0, 'a', 172.5, 'alpha', 0);
Link3 = Link('d', 0, 'a', 0, 'alpha', pi/2);
Link4 = Link('d', 107.8, 'a', 0, 'alpha', -pi/2);
Link5 = Link('d', 0, 'a', 0, 'alpha', pi/2);
Link6 = Link('d', 130.1, 'a', 0, 'alpha', 0);

ModelBot = SerialLink([Link1, Link2, Link3, Link4, Link5, Link6], 'name', 'MSEBOT');

% % Joint limits are defined (in terms of actual robot angles in degrees)
Q_UpperLimits = [250, 270, 240, 300, 260, 300, 210]; 
Q_LowerLimits = [0, 40, 30, 0, 55, 0, 140]; 

% Joint offsets from model to the actual robot
offset = [148.1 152.8 56.6 54.5 234.0 154.8];

% Points through path
Q_via = [ 146.3, 265.1, 50.7, 56.9, 255.7, 59.8, 207.3; % point #1
    146.3, 265.1, 67.4, 57.8, 69.2, 64.2, 185; % point #2
    146.3, 265.1, 81.8, 59.5, 71.3, 65.1, 185; % point #3
    146.3, 265.1, 81.8, 59.5, 71.3, 65.1, 210; % point #4
    146.3, 265.1, 50.7, 56.9, 255.7, 59.8, 210; % point #5
    147.2, 160.1, 47.8, 52.8, 243.4, 61.6, 210; % point #6
    148.1, 151.6, 63.6, 51.4, 167.7, 49.6, 210; % point #7
    144.3, 150.7, 67.2, 55.1, 68.9, 65.1, 210;% point #8
    143.1, 51.3, 56.6, 52.5, 68.9, 58.4, 210; % point #9
    144, 51.3, 145.7, 58.8, 132.8, 54.8, 210; % point #10
    144, 51.3, 145.7, 58.8, 132.8, 54.8, 185]; % point #11
len = length(Q_via);


%% Calculate function
% Loop through each configuration and find unknown variables
time = [0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20]; % time3
totalTrajectory = []; % holds the total trajectory of each point for each interval
TotalTrajectory = [];
%% Equation solving
allACoefficients = [];
for i=1:7 % for each theta
    % holds trajectory, velocity, acceleration, and time for each joint
    trajectoryJoint = [];
    velocityJoint = [];
    accelerationJoint = [];
    timeJoint = [];
    
    for j=1:(len-1) % for each position
        syms a0 a1 a2 a3 a4 a5
        
        % define initial conditions
        pi = Q_via(j,i);
        pf = Q_via(j+1,i);
        vi = 0;
        vf = 0;
        ai = 0;
        af = 0;
        ti = time(j);
        tf = time(j+1);
        
        % Set up equations
        eq1 = a0 + a1*ti + a2*ti^2 + a3*ti^3 + a4*ti^4 + a5*ti^5 == pi;
        eq2 = a0 + a1*tf + a2*tf^2 + a3*tf^3 + a4*tf^4 + a5*tf^5 == pf;
        eq3 = a1 + 2*a2*ti + 3*a3*ti^2 + 4*a4*ti^3 + 5*a5*ti^4 == vi;
        eq4 = a1 + 2*a2*tf + 3*a3*tf^2 + 4*a4*tf^3 + 5*a5*tf^4 == vf;
        eq5 = 2*a2 + 6*a3*ti + 12*a4*ti^2 + 20*a5*ti^3 == ai;
        eq6 = 2*a2 + 6*a3*tf + 12*a4*tf^2 + 20*a5*tf^3 == af;
        
        % Convert equations to matrix for solving
        [A,B] = equationsToMatrix([eq1, eq2, eq3, eq4, eq5, eq6], [a0, a1, a2, a3, a4, a5]);
        % Solve for coefficients
        X = linsolve(A,B);
        allACoefficients = [allACoefficients, X]; % all coefficients arranged in row
        
        % let's make it a row vector
        % sample for time
        sampleTime = ti:(tf-ti)/10:tf;
        timeJoint = [timeJoint,sampleTime];
        % calculate trajectory, velocity, acceleration
        trajectory = X(1) + X(2)*sampleTime + X(3)*sampleTime.^2 + X(4)*sampleTime.^3 + X(5)*sampleTime.^4 + X(6)*sampleTime.^5;
        trajectory = double(trajectory);
        % MODEL Variable to store joint trajectories for use later
        MJointTrajectory = [MJointTrajectory, trajectory];
        velocity = X(2) + 2*X(3)*sampleTime + 3*X(4)*sampleTime.^2 + 4*X(5)*sampleTime.^3 + 5*X(6)*sampleTime.^4;
        velocity = double(velocity);
        acceleration = 2*X(3) + 6*X(4)*sampleTime + 12*X(5)*sampleTime.^2 + 20*X(6)*sampleTime.^3;
        acceleration = double(acceleration);
        % combine into the trajectory for all time intervals of the joint
        trajectoryJoint = [trajectoryJoint,trajectory];
        velocityJoint = [velocityJoint,velocity];
        accelerationJoint = [accelerationJoint,acceleration];
    end
    % To hold total trajectory for each joint for model simulation
    TotalTrajectory = [TotalTrajectory; MJointTrajectory];
    %% Plot trajectory
    subplot(3,1,1);
    plot(timeJoint,trajectoryJoint);
    title("Position of Joint " + i);
    xlabel('Time (sec)');
    ylabel('Position (deg)');
    hold on;
    totalTrajectory = [totalTrajectory;trajectoryJoint]; % totalTrajectory holds trajectory where each row represents the trajectory of each joint
    % Plot velocity
    subplot(3,1,2)
    plot(timeJoint,velocityJoint);
    title("Velocity of Joint " + i)
    xlabel('Time (sec)');
    ylabel('Velocity (deg/sec)');
    hold on;
    % Plot acceleration
    subplot(3,1,3)
    plot(timeJoint,accelerationJoint);
    title("Acceleration of Joint " + i)
    xlabel('Time (sec)');
    ylabel('Acceleration (deg/sec^2)');
    hold on;
end

%% Simulation Model
TotalTrajectory = TotalTrajectory';
TotalTrajectory = TotalTrajectory(:,1:6);
MTotalTrajectory = TotalTrajectory - offset;
MTotalTrajectory(:,5) = MTotalTrajectory(:,5) * -1; % Joint 5 was rotating in wrong direction in model

figure(8)
for i=1:(len-1)*11 
    % Simulate each point stored in MTotalTrajectory
 	ModelBot.plot(deg2rad(MTotalTrajectory));
end


%% Initialize Robot and Points to Send
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');

coefficients = double(allACoefficients);

Q = totalTrajectory.'; % transpose totalTrajectory so each row represents a point

[points, ~] = size(Q); % Obtain number of points to send

Robot = MSE4401BOT(1234,4321); % Create robot object 

pause on; % Enable use of pause command

% Prepare the robot to move, set to maximum torque and low speed
disp('Ready to move the robot. Press a key to continue...');
pause;
Robot.sendTorque([1700, 1700, 1700, 1700, 1700, 1700, 1700]);
Robot.sendSpeed([5, 5, 5, 5, 5, 5, 5]); % can be increased to up to 5 for each joint

%% Sending the points to the robot 
for i = 1:points 
        Q_robot = Q(i,:); % Get the next point to be sent 
        
        % Convert from model angles to actual robot angles 
         Q_model = Q_robot; 
        
        % Check for being close to singularity using the model, only first 6 angles 
        J = ModelBot.jacob0(double(Q_model(1:6))); 
        DetJ = det(J); 
        if abs(DetJ) < 0.0001 
                Alarm_Singularity = 1; 
        else 
                Alarm_Singularity = 0; 
        end 
        
        % Check the joint limits 
        Alarm_Limits = 0; 
        for j = 1:7  
            if Q_robot(j) > Q_UpperLimits(j) || Q_robot(j) < Q_LowerLimits(j)
                    Alarm_Limits = 1; 
            end 
        end 
        
        Alarm = Alarm_Limits + Alarm_Singularity; % Any Fault in the system 
        
        % Send the point to the robot and wait for the robot to reach the point
        if Alarm == 0 
            Robot.sendPosition(Q_robot); 
            Q_current = transpose(Robot.getPosition); 
            while norm(Q_current - Q_robot,2) > 5 
                 Q_current = transpose(Robot.getPosition); 
            end 
        else 
            disp('The position is out of reach! Press enter to continue');
            pause;
        end 
end