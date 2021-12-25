close all
%% Init goals
%  Sets up goal trajectories for controller to use.

%% Create Simulink message bus. Don't forget to call rosinit before
% Check if the simulink model is open. If not, skip
if ~isempty(gcs)
    ros.createSimulinkBus(gcs)
end

%% Discrete goals
% Initialize left arm of the Baxter robot
states = baxEn();
%  Get starting position from Baxter's current position
bax_sub = rossubscriber('/robot/joint_states', rostype.sensor_msgs_JointState);
%  Get updates for all joints
msg = receive(bax_sub);
states = joint_states(msg,length(msg.Position),states);
initial_position = [0 states'];


%% Goals
%  Setup your goals and interpolate positions here
% goals = initial_position; Initial positions

%% YOUR CODE HERE
% Remember that your theta angles should be saved to a goals variable. The
% goals variable should be an [n by 17] matrix, where n = # of timesteps. 
% Each row of the goals matrix should be formatted as follows:
%   [timestep, left_s0, left_s1, left_e0, left_e1, left_w0, left_w1,...
%       left_w2, right_s0, right_s1, right_e0, right_e1, right_w0,...
%           right_w1, right_w2, left_gripper, right_gripper];

LGOpen = 0.015; % Open
LGClose = 0;
time = [0 6 12 18 24 30];

% Time, Left 7, Right 7, Left_Gripper, Right_Gripper
Goals = [0, 0.5832961945934286, 1.342233189399737, 0.29950974883462705, -0.6442719309118737, -0.5081311359870433, 0.9203884727312482, -3.0595246814374577, 0, 0, 0, 0, 0, 0, 0, LGOpen, 0;
        6, 0.7857816585943032, 1.7767332475682804, -0.25579129637989273, -0.8509758420794333, -0.7677573843366495, 0.8559612796400609, -3.0591411862404865, 0, 0, 0, 0, 0, 0, 0, LGClose, 0;
        12, 0.9552865356556414, 1.4227671807637212, -0.46824763550202253, -1.132844811853378, -0.44753889486556947, 1.3686943579907604, -3.058757691043515, 0, 0, 0, 0, 0, 0, 0, LGOpen, 0;
        18, 0.9694758579435815, 0.4456214188807127, -1.007825377640717, -0.9115680832009071, -0.5955680408965119, 1.951990552584189, -3.058757691043515, 0, 0, 0, 0, 0, 0, 0, LGClose, 0;
        24, 0.9142525495797066, 1.2532623037023831, -1.4837429170821665, -0.47553404424447826, -0.8931603137462821, 1.4166312576121796, -3.0579907006495723, 0, 0, 0, 0, 0, 0, 0, LGOpen, 0;
        30, 0.5721748338812593, 0.9997719785043184, -1.5301458359157003, 0.20286895919784598, -1.0162622719740866, 0.5227039534719548, -2.89385475634583320, 0, 0, 0, 0, 0, 0, 0, LGClose, 0];

[len, col] = size(Goals);

%% Calculate function
% Loop through each configuration and find unknown variables
totalTrajectory = []; % holds the total trajectory of each point for each interval
goals = [];
%% Equation solving
allACoefficients = [];
for i=1:col % for each theta
    % holds trajectory, velocity, acceleration, and time for each joint
    trajectoryJoint = [];
    velocityJoint = [];
    accelerationJoint = [];
    timeJoint = [];
    
    for j=1:(len-1) % for each position
        syms a0 a1 a2 a3 a4 a5
        
        % define initial conditions
        pi = Goals(j,i);
        pf = Goals(j+1,i);
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
        sampleTime = ti:(tf-ti)/100:tf;
        timeJoint = [timeJoint,sampleTime];
        % calculate trajectory, velocity, acceleration
        trajectory = X(1) + X(2)*sampleTime + X(3)*sampleTime.^2 + X(4)*sampleTime.^3 + X(5)*sampleTime.^4 + X(6)*sampleTime.^5;
        trajectory = double(trajectory);
        % MODEL Variable to store joint trajectories for use later
        velocity = X(2) + 2*X(3)*sampleTime + 3*X(4)*sampleTime.^2 + 4*X(5)*sampleTime.^3 + 5*X(6)*sampleTime.^4;
        velocity = double(velocity);
        acceleration = 2*X(3) + 6*X(4)*sampleTime + 12*X(5)*sampleTime.^2 + 20*X(6)*sampleTime.^3;
        acceleration = double(acceleration);
        % combine into the trajectory for all time intervals of the joint
        trajectoryJoint = [trajectoryJoint,trajectory];
        velocityJoint = [velocityJoint,velocity];
        accelerationJoint = [accelerationJoint,acceleration];
    end
    %totalTrajectory = [totalTrajectory;trajectoryJoint];
    goals = [goals, trajectoryJoint'];
    totalTrajectory(1,1:11) = 0:0.4:4;
    totalTrajectory(1,12:22) = 4:0.4:8;
    totalTrajectory(1,23:33) = 8:0.4:12;
    totalTrajectory(1,34:44) = 12:0.4:16;
    totalTrajectory(1,45:55) = 16:0.4:20;
    
    %% Plot trajectory
    if( i>=2 && i<=8)
        figure
        subplot(3,1,1);
        plot(timeJoint,trajectoryJoint);
        title("Position of Joint " + i);
        xlabel('Time (sec)');
        ylabel('Position (deg)');
        hold on;
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
end
goals(:,3)=abs(goals(:,3));
goals(:,5)=abs(goals(:,5));
Kp = [  1,0,0,0,0,0,0;... 
        0,1,0,0,0,0,0;... 
        0,0,1,0,0,0,0;... 
        0,0,0,1,0,0,0;... 
        0,0,0,0,1,0,0;... 
        0,0,0,0,0,1,0;... 
        0,0,0,0,0,0,1];
Kv = [  0,0,0,0,0,0,0;...
        0,0,0,0,0,0,0;...
        0,0,0,0,0,0,0;... 
        0,0,0,0,0,0,0;... 
        0,0,0,0,0,0,0;... 
        0,0,0,0,0,0,0;... 
        0,0,0,0,0,0,0]; 

%-------------------------------------------------------------------------
% Uncomment this block to test the Simulink model using a set
% of sample goals. Do not forget to comment this block again when  
% solving the assignment.
 
% load('sampleGoals.mat')
% goals = sampleGoals;

%-------------------------------------------------------------------------