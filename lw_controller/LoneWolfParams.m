%% Lone Wolf simulation parameters

% Total mass of vehicle
m = 375;
len = 2.39;
with = 1.22;
% Moment of inertia of vehicle
Iz = (1/12) * m * (with^2 + len^2);

% Distance from CoG to front axle
a = 0.9;

% Distance from CoG to rear axle
b = 0.9;

% Initial velocity in x direction
xdotinit = 1e-3;

% Gravity
g = 9.81;

wheelRadius = 0.33;

% same as in mpc controller class file. This variable does not do anything
% because i am not allowed to use workspace variables in simulink?!?!?!

% This number need to be in the "max length" of messagetype
% "float64multiarray"
% in "variable size messages", under "simulation" tab

% Also in the functions "loop reference" under "inputs" subsystem
N_STEPS_REFERANCE = 500; 

% magic tire model
B = 10;
C = 1.9;
D = 1;
E = 1;

