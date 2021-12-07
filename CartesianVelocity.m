function CV = CartesianVelocity(jointVelocities, DHConfig)
    %CartesianVelocity computes the Cartesian velocity of the end effector,
    %given a set of joint velocities and the manipulator's configuration
    
    jointCount = size(DHConfig,1); % Number of joints
    w = cell(jointCount+1,1); % initialize angular velocity
    w{1} = [0;0;0]; %fixed base
    v = cell(jointCount+1,1); % initialize linear velocity
    v{1} = [0;0;0]; %fixed base
    RM_0N = eye(jointCount); % initial rotation matrix from end-effector to ground
    
    for i = 1 : jointCount
        % Retrieve joint velocities and manipulator's configuration
        a = DHConfig(i,1);
        alpha = DHConfig(i,3);
        d = DHConfig(i,2);
        theta = DHConfig(i,4);
        dtheta = jointVelocities(i);
        dd = jointVelocities(2,i);
        
        TM = TransformMatrix(a, alpha, d, theta); % Calculate Transformation matrix
        RM = TM(1:3,1:3); % Rotation matrix
        RMi = RM.'; % Rotation matrix inverse by transpose
        RM_0N = RM_0N * RM; % Update final Rotation matrix
        P = TM(1:3, 4); % Position
        
        if d == 0 % Revolute joint
            w{i+1} = RMi*w{i}+ [0; 0; dtheta]; % calculate angular velocity
            v{i+1} = RMi*(v{i}+cross(w{i},P)); % calculate linear velocity
        else % Prismatic joint
            w{i+1} = RM*w{i}; % calculate angular velocity
            v{i+1} = RM*(v{i} + cross(w{i},P)) + [0;0;dd]; % calculate linear velocity
        end  
    end
    
    AngularV = RM_0N * w{jointCount + 1}; % calculate angular velocity
    LinearV = RM_0N * v{jointCount + 1}; % calculate linear velocity
    CV = [LinearV; AngularV]; % Cartesian velocity matrix
end
