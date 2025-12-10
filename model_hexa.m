% run_hex.m
clear; clc;
% cargar params
params = get_hex_params();

% initial state
p0 = [0;0;0];
v0 = [0;0;0];
q0 = [1;0;0;0]; % identidad (w,x,y,z)
omega0 = [0;0;0];
x0 = [p0; v0; q0; omega0];

% objetivo
z_des = 1.0; % m
yaw_des = 0;

% controlador simple: altitude PID -> total thrust; attitude PD -> distribute
Kp_z = 20; Ki_z = 5; Kd_z = 5;
Kp_att = diag([8,8,4]); Kd_att = diag([2,2,1]);

% integrator
int_z = 0;
prev_err_z = 0;

tspan = 0:0.01:10;
x = x0;
X = zeros(length(tspan), length(x0));
W = zeros(length(tspan),6);

for k=1:length(tspan)
    t = tspan(k);
    % read state
    p = x(1:3); v = x(4:6); q = x(7:10); omega = x(11:13);
    z = p(3);
    % ALTITUDE PID -> desired total thrust
    err_z = z_des - z;
    int_z = int_z + err_z*0.01;
    der_z = (err_z - prev_err_z)/0.01;
    prev_err_z = err_z;
    % desired total force along body z (approx)
    F_total = params.m * (params.g + Kp_z*err_z + Ki_z*int_z + Kd_z*der_z);
    % desired body torques from attitude error (simple: keep level)
    % extract roll/pitch from quaternion (small angle)
    R = quat2rotm(q');
    % compute current euler (roll pitch yaw)
    eul = rotm2eul(R,'XYZ'); % [roll pitch yaw]
    err_att = -eul'; % desire 0 roll,pitch, yaw_des
    err_att(3) = wrapToPi(yaw_des - eul(3));
    tau_des = Kp_att * err_att + (-Kd_att)*omega;
    
    % allocation: solve for w^2 given total thrust and tau_des
    % unknowns: s = w.^2 (6x1): linear system A*s = b
    A = zeros(3,6); % tau_x, tau_y, tau_z contributions per rotor
    % tau_x contribution = y_i * kf
    A(1,:) = params.kf * params.rpos(2,:);
    A(2,:) = -params.kf * params.rpos(1,:);
    A(3,:) = params.km * params.sgn;
    % thrust eq
    A_thrust = params.kf * ones(1,6);
    % create augmented solve: [A_thrust; A] * s = [F_total; tau_des]
    M = [A_thrust; A];
    b = [F_total; tau_des];
    % least-squares with nonnegativity
    s = lsqnonneg(M, b); % requires Optimization Toolbox; if not available use pinv then max(0,...)
    % get w
    w = sqrt(max(s,0));
    % integrate one step with ode45 small step
    fun = @(tt,xx) hex_dynamics(tt, xx, w, params);
    [ttt, xxout] = ode45(fun, [t t+0.01], x);
    x = xxout(end,:)';
    X(k,:) = x';
    W(k,:) = w';
end

% Plot z
figure; plot(tspan, X(:,3)); xlabel('t [s]'); ylabel('z [m]'); grid on;
figure; plot(tspan, W); xlabel('t [s]'); ylabel('w [rad/s]'); legend('w1','w2','w3','w4','w5','w6');
