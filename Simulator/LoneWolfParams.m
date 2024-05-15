%% Lone Wolf simulation parameters

m = 375; % Total mass of vehicle
len = 2.39; % Length of vehicle
width = 1.22; % Width of vehicle
a = len/2; % Center of gravity to front axle
b = len/2; % Center of gravity to rear axle
Iz = (1/12) * m * (width^2 + len^2); % Moment of inertia of vehicle
Iw = 0.5; % Moment on inertia of wheel
xdotinit = 1e-3; % Initial velocity in x-direction
g = 9.81; % Gravitational acceleration
wheelRadius = 0.33; % Radius of the wheels
RRC = 5; % Rolling resistance coefficient

% This number need to be in the "max length" of messagetype
% "float64multiarray"
% in "variable size messages", under "simulation" tab.
N_STEPS_REFERENCE = 500;

% Pacejka Tire model

% Longitudinal Pacejka Parameters
Bl = 10; %6
Cl = 1.9;
Dl = 1*g*m/2;
El = 0.97;

% Lateral Pacejka Parameters
Bc = 10;
Cc = 1.9;
Dc = 1*g*m/2;
Ec = 0.97;

% Example values
% Dry: 10, 1.9, 1, 0.97;
% Wet: 12, 2.3, 0.82, 1
% Ice: 4, 2, 0.1, 1

% Kappa and alpha peak (calculate from Pacejka curves)
kappa_peak = 0.18;
alpha_peak = 0.18;

% Gear paramters
velLimitLow = 3.72;   % Lower speed that the CVT (transmittion) starts to kicks in
velLimitHigh = 12.0;    % Upper speed that the CVT is completly engaged
lowRatio = 5.952;     % Starting gear ratio
highRatio = 3.691;    % Final gear ratio
FinalDrive_gear_ratio = 4.04; % Scalar for output RPM to wheels
Carengine_to_atv_engine_scalar_magicnumber = 0.4545;