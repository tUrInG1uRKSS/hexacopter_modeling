function xdot = hex_dynamics(t, x, w, params)
% x = [p(3); v(3); q(4); omega(3)]
p = x(1:3);
v = x(4:6);
q = x(7:10); q = q / norm(q);
omega = x(11:13);

m = params.m; g = params.g; I = params.I;
kf = params.kf; km = params.km;
rpos = params.rpos; sgn = params.sgn;

% rotor thrusts
T = kf * (w.^2); % 6x1

% total thrust (body frame, z axis positive upward in body)
Fz_body = sum(T); % thrust along body z (assume rotors push -z body to go up; sign convention)
F_body = [0; 0; Fz_body];

% Rotation matrix from body to inertial using quaternion
R = quat2rotm(q'); % MATLAB expects row vector q' maybe; adjust if needed

% Translational acceleration (inertial)
p_dd = (1/m) * (R * F_body) + [0;0;-g];

% Torques: roll/pitch from thrust-offset, yaw from rotor reaction
tau = zeros(3,1);
% tau_x = sum( y_i * T_i ) ; tau_y = -sum( x_i * T_i )  (right-hand rule)
tau(1) = sum( rpos(2,:) .* T );  % roll
tau(2) = -sum( rpos(1,:) .* T ); % pitch
tau(3) = km * sum( sgn .* (w.^2) ); % yaw

% Angular acceleration
omega_dot = I \ (tau - cross(omega, I*omega));

% Quaternion derivative
Omega = [ 0     -omega(1) -omega(2) -omega(3);
          omega(1) 0      omega(3) -omega(2);
          omega(2) -omega(3) 0      omega(1);
          omega(3) omega(2) -omega(1) 0];
q_dot = 0.5 * Omega * q;

xdot = [v; p_dd; q_dot; omega_dot];
end
