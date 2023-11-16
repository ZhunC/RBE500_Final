clear 
clc

theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;

% Link 0
a0 = 0;
alpha0 = 0;
d0 = 36.076;
t0 = 0;

% Link 1
a1 = 0;
alpha1 = -pi/2;
d1 = 60.25;
t1 = theta1;

% Link 2
a2 = 128;
alpha2 = 0;
d2 = 0;
t2 = theta2 - pi/2;

% Link 3
a3 = 148;
alpha3 = 0;
d3 = 0;
t3 = theta3 + pi/2;

% Link 4
a4 = 133.4;
alpha4 = 0;
d4 = 0;
t4 = theta4;

H0_1 = DH(a0,alpha0,d0,t0);
H1_2 = DH(a1,alpha1,d1,t1);
H2_3 = DH(a2,alpha2,d2,t2);
H3_4 = DH(a3,alpha3,d3,t3);
H4_5 = DH(a4,alpha4,d4,t4);

H0_5_num = H0_1 * H1_2 * H2_3 * H3_4 * H4_5


%% type in or use existing transformation matrix H0_5 from workspace 
% which is the current end effector frame expressed in the space frame
% length in mm
% a user-defined H0_5 for testing: 
% H0_5 = [[1, 0, 0, 0];...
%     [0, 1, 0, 0];...
%     [0, 0, 1, 300];...
%     [0, 0, 0, 1]];

%%
H0_5 = H0_5_num;

%% IK
t1 = atan2(-H0_5(1,3),H0_5(2,3)); % theta 1 solved
t234 = atan2(-H0_5(3,1),-H0_5(3,2)); % theta 2+3+4

% solving for theta 2 and theta 3 simultaneously
f = @(x) assistive(x,H0_5,t1,t234);
x0 = [t2, t3];
% this function fsolve should have its counterpart in python as...
% scipy.optimal.fsolve
sol = fsolve(f,x0); % sol should be in form [theta2, theta3]

t2 = sol(1); % theta 2 solved
t3 = sol(2); % theta 3 solved
t4 = t234-t2-t3; % theta 4 solved

thetaList = [t1, t2+pi/2, t3-pi/2, t4]


function t = assistive(x, H0_5, t1, t234)
% x(1) = theta2, x(2) = theta3
% px entry from H:
t(1) = (cos(t1)*(667*cos(t234) + 740*cos(x(1) + x(2)) + 640*cos(x(1))))/5 - H0_5(1,4);
% pz entry from H:
t(2) = 48163/500 - 148*sin(x(1) + x(2)) - 128*sin(x(1)) - (667*sin(t234))/5 - H0_5(3,4);
end
