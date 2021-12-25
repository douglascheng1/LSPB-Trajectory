% Lab 2 Code Inverse Kinematics

L1 = Link('d', 117.6, 'a', 0, 'alpha', pi/2);
L2 = Link('d', 0, 'a', 172.5, 'alpha', 0);
L3 = Link('d', 0, 'a', 0, 'alpha', pi/2);
L4 = Link('d', 107.8, 'a', 0, 'alpha', -pi/2);
L5 = Link('d', 0, 'a', 0, 'alpha', pi/2);
L6 = Link('d', 130.1, 'a', 0, 'alpha', 0);

%Define Bot
bot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'my robot');

C1 = ForwardKinematicsEquation(0,0,90,0,0,0) % Configuration 1
C2 = ForwardKinematicsEquation(0,90,0,0,0,0) % Configuration 2
L = [117.6 172.5 107.8 130.1];

% C1 = [0,0,1,410.4;0,-1,0,0;1,0,0,117.6;0,0,0,1];
% C2 = [-1,0,0,0;0,-1,0,0;0,0,1,528;0,0,0,1];

%Solve inverse kinematics using robotics toolbox
toolboxC1 = bot.ikine(C1,'pinv')
toolboxC2 = bot.ikine(C2,'pinv')

%Solve inverse kinematics using invkinCC2
invC1 = invkinCC(C1)
invC2 = invkinCC(C2)

%Plot both configurations
bot.plot(invC1); %try robot toolbox vals too
bot.plot(invC2)

