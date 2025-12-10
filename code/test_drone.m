clear all; close all; clc;

%% ---------------------------------------------------------------
%                       USER SETTINGS
% ---------------------------------------------------------------
% Case 0: HOVER TEST 
%         The drone should hover and stay perfectly still at [0,0,0] 
% Case 1: YAW ROTATION 
%         The drone should hover (vz ~ 0) and start rotating in yaw (psi > 0 and r > 0) 
% Case 2: VERTICAL TAKEOFF 
%         The drone should accelerate and go up vertically (vz > 0) 
% Case 3: FORWARD PITCH -> FORWARD MOVEMENT +X 
%         The drone should tilt "nose down" (theta > 0 and q > 0) and start moving forward (vx > 0) 
% Case 4: LEFTWARD TILT -> LEFT MOVEMENT 
%         The drone should tilt to the left (phi < 0 and p < 0) and start moving/gliding to the left (vy > 0) 
% Case 5: TEST FOR GIMBAL LOCK -> [phi theta psi] = [0 pi/2 0]
%         With the Euler model, the simulaion fails. With the quaternions model, it works 
% Case 6: TAKEOFF AND YAW ROTATION 
%         Thrust > Gravity, the drone should go up (vz > 0) and should rotate in yaw (psi > 0)
test_case   = 1;         % Choose case 0–6
model_type  = 'euler';    % 'euler'  or  'quat'
animate_flag = true;     % Show animation or not
tf = 2;                  % Simulation time
dt = 1e-2;               % Time step
%% ---------------------------------------------------------------


%% ------------ 1) DRONE PARAMETERS -----------------------------
PARAMS.m = 2.00; % cylinder mass (kg) 
PARAMS.D = 0.20; % cylinder diameter (m) 
PARAMS.H = 0.04; % cylinder height (m) 
PARAMS.L = 0.30; % arms lenght (m)
PARAMS.l = 0.25; % propeller lenght (m) 
PARAMS.Im = 0.00; % Inertia moment of one motor 
PARAMS.Ir = 3.357e-5; % Rotational inertial for each propeller 

PARAMS.alpha = [1;-1;1;-1;1;-1]; % rotations directions of motors (CCW, CW ...) 
PARAMS.beta = ones(6,1); % thrust forces durections 

PARAMS.g = 9.81; % gravity acceleration (m/s^2) 
PARAMS.rho = 1.293; % air density (kg/m^3) 

PARAMS.CT = 1e-5; % Thrust or lift constant (force) 
PARAMS.CD = 1e-7; % Torque constant (due to drag force) 

% Defining the inertia matrix for the drone (asummed as a cylinder) 
Ixx = (1/12) * PARAMS.m * (PARAMS.H^2 + 3*(PARAMS.D/2)^2); % Inertia around x-axis 
Iyy = (1/12) * PARAMS.m * (PARAMS.H^2 + 3*(PARAMS.D/2)^2); % Inertia around y-axis 
Izz = (1/2) * PARAMS.m * (PARAMS.D/2)^2; % Inertia around z-axis 
PARAMS.I = diag([Ixx, Iyy, Izz]); 

% Calculating constant for forces and torques 
PARAMS.k = PARAMS.CT*PARAMS.rho*pi*PARAMS.l^4; % Ti = k*(wi)^2 
PARAMS.b = PARAMS.CD*PARAMS.rho*pi*PARAMS.l^4; % Ci = b*(wi)^2

%% ------------ 2) INPUTS FOR EACH TEST CASE ---------------------
[TB, x0] = generate_test_case(test_case, PARAMS);

% Convert thrust to motor speeds
u  = 1/PARAMS.k*TB; 
wm = sqrt(u);
% Here we considere that the inputs of the system will be the squared 
% angular velocities because forces and torques are proportial to them

%% ------------ 3) CHOOSE DYNAMICS MODEL ------------------------
switch model_type
    case 'euler'
        dyn_f = @(t,x) dynamics_model_euler(t, x, u, PARAMS);
    case 'quat'
        dyn_f = @(t,x) dynamics_model_quat(t, x, u, PARAMS);
        ang = x0(10:12);
        quat0 = eul2quat(ang');
        x0 = [zeros(3,1); zeros(3,1); zeros(3,1); quat0']; 
    otherwise
        error("model_type must be 'euler' or 'quat'");
end

%% ------------ 4) SIMULATION ----------------------------------
tspan = 0:dt:tf;
[t, X] = ode45(dyn_f, tspan, x0);

positions = X(:,1:3);

if strcmp(model_type,'euler')
    angles = X(:,10:12);
else
    q = X(:,10:13);
    angles = quat2eul(q,"XYZ");
end

%% ------------ 5) PLOTS FOR ANALYSIS ---------------------------
plot_all(t, X, angles, model_type);

%% ------------ 6) ANIMATION -----------------------------------
if animate_flag
    if strcmp(model_type,'euler')
        animations(positions, angles, PARAMS, 'euler');
    else
        animations(positions, q, PARAMS, 'quat');
    end
end
