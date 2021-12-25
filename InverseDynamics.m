mass = [4; 2; 2]; % mass
g = 9.8; % gravity constant
h = 0.4;
r = 0.2;
jointCount = 3; % number of joints

% trajectory symbols
time = sym('time'); %time 
position = sym('posistion', [1,1,3]); %posistion
velocity = sym('velocity', [1,1,3]); %velocity 
acceleration = sym('acceleration', [1,1,3]); %acceleration

% define given position functions 
position(:,:,1) = deg2rad(45)*(1+6*exp(-time/0.6)-8*exp(-time/0.8)); % revolute joint 1
position(:,:,2) = deg2rad(45)*(1+6*exp(-time/0.6)-8*exp(-time/0.8)); % revolute joint 2
position(:,:,3) = 0.4*(1+6*exp(-time/0.6)-6*exp(-time/0.8)); % prismatic joint 3

for i=1:jointCount % calculate velocity functions
    velocity(:,:,i) = diff(position(:,:,i));
end 
 
for i=1:3 % calculate acceleration functions
    acceleration(:,:,i) = diff(velocity(:,:,i)); 
end 

%Inverse dynamics symbols
P_c = sym('Pc',[3,1,3]); % vectors for COM
I_c = sym('I',[1,1,3]); % inertial tensor 
linkLength = sym('l',[3,1]); % length of links
linkForce = sym('f',[3,1,4]); % force on link i by i-1
linkForce_c = sym('fc', [3,1,3]); % force at CoM of link
linkTorque = sym('n',[3,1,4]); % torque on link i by i-1
linkTorque_c = sym('N', [3,1,3]); % torque at CoM of link
angularV = sym('omega',[3,1,4]); % angular velocity
angularA = sym('omegaDot',[3,1,4]); % angular acceleration
linearA = sym('nuDot',[3,1,4]); % linear acceleration
linearA_c = sym('nuDotC',[3,1,4]); % linear acceleration at CoM of link
tau = sym('tau',[1,1,3]); % final joint torques

% unit vectors 
x_unit = [1; 0; 0];
y_unit = [0; 1; 0];
z_unit = [0; 0; 1];

% Vectors locating COM
P_c(:,:,1)=0;
P_c(:,:,2)=0;
P_c(:,:,3)=0;

% Inertia tensors all = 0 because
I_c(:,:,1)=0;
I_c(:,:,2)=0;
I_c(:,:,3)=0;

% Assume no forces on the end-effector 
linkForce(:,:,4)=0;
linkTorque(:,:,4)=0; 

% base of the robot is not rotating 
angularV(:,:,1)=0; 
angularA(:,:,1)=0; 

% include gravity forces 
linearA(:,:,1)=g*z_unit;

% Position vectors from frame i to i+1
P=sym('P',[3,1,3]);
P(:,:,1)=[0;0;linkLength(1)];
P(:,:,2)=[0;linkLength(2);0];
P(:,:,3)=[0;-linkLength(3);0];

% OUTWARD ITERATIONS 
% Generate rotation matrices
% DH parameters
alpha = [0 -90 90 0];
% symbols for theta
theta = sym('theta',[1,1,4]); 
dtheta = sym('thetaDot',[1,1,4]);
ddtheta = sym('thetaDotDot',[1,1,4]);
% symbols for d - required for prismatic joints
d = sym('d',[1,1,3]);
dDot = sym('dDot',[1,1,3]);
dDotDot = sym('dDotDot',[1,1,3]);
% rotation matrices
R = sym('R',[3,3,4]); 
for i=1:jointCount+1
    R(:,:,i) = [cos(theta(i)), -sin(theta(i)), 0; 
        sin(theta(i))*cos(alpha(i)), cos(theta(i))*cos(alpha(i)), -sin(alpha(i)); 
        sin(theta(i))*sin(alpha(i)), cos(theta(i))*sin(alpha(i)), cos(alpha(i))];
end

for i=1:3
    % angular velocity 
    angularV(:,:,i+1)=transpose(R(:,:,i))*angularV(:,:,i)+dtheta(:,:,i)*z_unit;
    % angular acceleration 
    if (i+1) == 4 % if pismatic 
        angularA(:,:,i+1)=transpose(R(:,:,i))*angularV(:,:,i); 
    else % else it is revolute
        angularA(:,:,i+1)=transpose(R(:,:,i))*angularA(:,:,i)+cross(transpose(R(:,:,i))*angularV(:,:,i),dtheta(:,:,i)*z_unit)+ddtheta(:,:,i)*z_unit;
    end    
    % linear acceleration 
    if (i+1) == 4 % if pismatic 
        linearA(:,:,i+1)=transpose(R(:,:,i))*(cross(angularA(:,:,i),P(:,:,i))+cross(angularV(:,:,i),cross(angularV(:,:,i),P(:,:,i)))+linearA(:,:,i))+cross(2*angularV(:,:,i+1),dDot(:,:,i)*z_unit)+dDotDot(:,:,i)*z_unit;
    else % else it is revolute
        linearA(:,:,i+1)=transpose(R(:,:,i))*(cross(angularA(:,:,i),P(:,:,i))+cross(angularV(:,:,i),cross(angularV(:,:,i),P(:,:,i)))+linearA(:,:,i));
    end    
    % linear acceleration about CoM
    linearA_c(:,:,i+1)=cross(angularA(:,:,i+1),P_c(:,:,i))+cross(angularV(:,:,i+1),cross(angularV(:,:,i+1),P_c(:,:,i)))+linearA(:,:,i+1);
    % F at CoM of Link
    linkForce_c(:,:,i)=mass(i)*linearA_c(:,:,i+1);
    % N at CoM of Link
    linkTorque_c(:,:,i)=I_c(:,:,i)*angularA(:,:,i+1)+cross(angularV(:,:,i+1),I_c(:,:,i)*angularV(:,:,i+1));
end 

for i=3:-1:1
    linkForce(:,:,i)=R(:,:,i+1)*linkForce(:,:,i+1)+linkForce_c(:,:,i); % force on links
    linkTorque(:,:,i)=linkTorque_c(:,:,i)+R(:,:,i+1)*linkTorque(:,:,i+1)+cross(P_c(:,:,i),linkForce_c(:,:,i))+cross(P(:,:,i),R(:,:,i+1)*linkForce(:,:,i+1)); % moment on links
    % joint torques 
    if i==3 % case of prismatic
        tau(:,:,i)=transpose(linkForce(:,:,i))*(z_unit);
    else % revolute 
        tau(:,:,i)=transpose(linkTorque(:,:,i))*(z_unit);
    end
end

% sub in for theta, thetaDot, thetaDotDot
tau(:,:,:)=subs(tau(:,:,:),{theta(1), theta(2), theta(3)},{position(1), position(2), 0});
tau(:,:,:)=subs(tau(:,:,:),{dtheta(1), dtheta(2), dtheta(3)},{velocity(1), velocity(2), 0});
tau(:,:,:)=subs(tau(:,:,:),{ddtheta(1), ddtheta(2), ddtheta(3)},{acceleration(1), acceleration(2), 0});
tau(:,:,:)=subs(tau(:,:,:),{d(3), dDot(3), dDotDot(3)},{position(3), velocity(3), acceleration(3)});
tau(:,:,:)=subs(tau(:,:,:),{linkLength(1), linkLength(2), linkLength(3)},{h, r, position(3)});

%Plots
time = 1:0.01:3;
for i=1:3 % joint number 
    figure
    title(['Joint ' num2str(i)])
    
    subplot(4,1,1)
    plot(time,subs(position(i))) % position plot
    title('Position')
    xlabel('t')
    ylabel('rad')
    if i == 3
        ylabel('m')
    end 
 
    subplot(4,1,2)
    plot(time,subs(velocity(i))) % velocity plot
    title('Velocity')
    xlabel('t')
    ylabel('rad/s')
    if i == 3
        ylabel('m/s')
    end 
    
    subplot(4,1,3)
    plot(time,subs(acceleration(i))) % acceleration plot
    title('Acceleration')
    xlabel('t')
    ylabel('rad/s^2')
    if i == 3
        ylabel('m/s^2')
    end 
    
    subplot(4,1,4)
    plot(time,subs(tau(:,:,i))) % joint torque plot
    title('Joint Torque')
    xlabel('t')
    ylabel('Nm')
    if i == 3
        ylabel('N')
    end 
    sgtitle(['Joint ' num2str(i)])
end