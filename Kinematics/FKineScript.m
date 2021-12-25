% Lab 2 Code Forward Kinematics

L1 = Link('d', 117.6, 'a', 0, 'alpha', pi/2);
L2 = Link('d', 0, 'a', 172.5, 'alpha', 0);
L3 = Link('d', 0, 'a', 0, 'alpha', pi/2);
L4 = Link('d', 107.8, 'a', 0, 'alpha', -pi/2);
L5 = Link('d', 0, 'a', 0, 'alpha', pi/2);
L6 = Link('d', 130.1, 'a', 0, 'alpha', 0);


% C1 = 0,90
% C2 = 90,0
theta1 = 0, theta2 = 90, theta3 = 0, theta4 = 0, theta5 = 0, theta6 = 0;

ThetaVals = [theta1*(pi/180) theta2*(pi/180) theta3*(pi/180) theta4*(pi/180) theta5*(pi/180) theta6*(pi/180)];

bot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'my robot'); 

DH = [0, 117.6, 90, theta1; 
    172.5, 0, 0, theta2; 
    0, 0, 90, theta3; 
    0, 107.8, -90, theta4; 
    0, 0, 90, theta5; 
    0, 130.1, 0, theta6]

T = bot.fkine([theta1*(pi/180) theta2*(pi/180) theta3*(pi/180) theta4*(pi/180) theta5*(pi/180) theta6*(pi/180)])
x4 = ForwardKinematicsEquation(theta1, theta2, theta3, theta4, theta5, theta6)

bot.plot(ThetaVals);
%bot.plot([-0.0000    0.3958    0.3838    0.0000    0.7911    0.0000]);

%qi1 = bot.ikine(T)

%qi2 = invkinCC(T)
%T1 = bot.fkine(qi2)

