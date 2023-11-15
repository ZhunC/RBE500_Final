% Referenced: Ting Han, Zhong, et al. Kinematic analysis for trajectory planning of open source 4 DOF robot arm 
% link below
% https://www.researchgate.net/publication/353075915_Kinematic_Analysis_for_Trajectory_Planning_of_Open-Source_4-DoF_Robot_Arm

clear 
clc

%% type in or use existing transformation matrix H0_5 from workspace 
% which is the current end effector frame expressed in the space frame
% length in mm
% a user-defined H0_5 for testing: 
H0_5 = [[1, 0, 0, 300];...
    [0, 1, 0, 0];...
    [0, 0, 1, 0];...
    [0, 0, 0, 1]];
d1 = 36.076 + 60.25; % mm, length of link 0+link 1
a2 = sqrt(128^2+24^2); % mm, length between J2 and J3
a3 = 124; % mm, length of link 3
a4 = 133.4; % mm, length of the end effector 


%% IK
% the same notation is used as much as possible with the original paper

% theta 1
t1 = atan2(-H0_5(1,3),H0_5(2,3)); % theta 1 solved

% r axis transformation
phi = atan2(-H0_5(3,1),-H0_5(3,2)); % theta 2+3+4, Eq. 16
px = H0_5(1,4);
py = H0_5(2,4);
r3 = sqrt(px^2 + py^2); % Eq. 13, 14
z3 = H0_5(3,4) - d1; % Eq. 15

% joint 4 location in r-z plane
r2 = r3 - a4*cos(phi); % Eq. 17
z2 = z3 - a4*sin(phi); % Eq. 18

% theta 3 from  Eq. 20
t3_1 = acos((r2^2+z2^2-a2^2-a3^2) / (2*a2*a3)); % first pose solution of theta 3
t3_2 = -acos((r2^2+z2^2-a2^2-a3^2) / (2*a2*a3)); % second pose solution of theta 3
t3 = [t3_1, t3_2];

% theta 2
c2 =((a2+a3*cos(t3))*r2 + a3*sin(t3)*z2 ) / (r2^2+z2^2); % Eq. 25
s2 =((a2+a3*cos(t3))*z2 + a3*sin(t3)*r2 ) / (r2^2+z2^2); % Eq. 26
t2 = atan2(s2,c2); % Eq. 27

% theta 4
t4 = phi-t2-t3; % Eq. 28

thetaList = [[t1, t1];...
    t2;
    t3;
    t4]


