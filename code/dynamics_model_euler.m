function dx = dynamics_model_euler(~, x, u, PARAMS)
% dynamics_model_euler: Dynamics of a hexacopter (Euler ZYX convention)
% Inputs:
%   ~      : time (not used here, required by ODE solvers)
%   x      : state-vector (12x1) = [px; py; pz; vx; vy; vz; p; q; r; phi; theta; psi]
%            (Positions, velocities, body angular rates, Euler angles)
%   u      : motor inputs (6x1) = [w1^2; ...; w6^2] (squared rotor speeds)
%   PARAMS : structure with required fields:
%            .m      mass
%            .g      gravity
%            .k      thrust coefficient (N per w^2)
%            .beta   per-rotor thrust scale (6x1) optional (use ones if not)
%            .L      arm length (distance rotor->CG)
%            .b      rotor moment coefficient (Nm per w^2)
%            .alpha  rotor spin direction signs (6x1: +1 or -1)
%            .Ir     rotor inertia about spin axis (scalar)
%            .I      body inertia matrix (3x3)
%
% Output:
%   dx : derivative of state-vector (12x1)

% -------------------------
% Safety / defaults checks
% -------------------------
if ~isfield(PARAMS,'beta'); PARAMS.beta = ones(6,1); end
if ~isfield(PARAMS,'alpha'); PARAMS.alpha = [+1;-1;+1;-1;+1;-1]; end

% -------------------------
% Extract states
% -------------------------
px = x(1); py = x(2); pz = x(3); % (unused but kept for clarity)
vx = x(4); vy = x(5); vz = x(6);
omega = x(7:9);   % [p; q; r]
phi = x(10); theta = x(11); psi = x(12);

p = omega(1); q = omega(2); r = omega(3);

% -------------------------
% Rotation matrix (body -> inertial) ZYX
% -------------------------
% Uses intrinsic rotations: R = Rz(psi) * Ry(theta) * Rx(phi)
R = rotz(psi)*roty(theta)*rotx(phi);
% Many MATLAB toolboxes implement rotX/rotY/rotZ in degrees
% We have custom functions that accept radians

% -------------------------
% Thrust forces
% -------------------------
% Protect from tiny negative numerical values inside sqrt (u should be >=0)
u = max(u, 0);
T_i = PARAMS.k*(PARAMS.beta(:).*u(:)); % thrust per rotor (N)

% -------------------------
% Translational dynamics (in inertial frame)
% -------------------------
V = [vx; vy; vz];
dV = [0;0;-PARAMS.g] + (1/PARAMS.m)*(R*[0;0;sum(T_i)]);

% -------------------------
% Euler kinematics: relation between body rates and Euler derivatives
% -------------------------
% Wn_inv = inv(W) where deuler = W^{-1} * omega_body
Wn = [1     0         -sin(theta);
      0  cos(phi) cos(theta)*sin(phi);
      0 -sin(phi) cos(phi)*cos(theta)]; 

Wn_inv = [ 1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
           0,     cos(phi),           -sin(phi);
           0, sin(phi)/cos(theta), cos(phi)/cos(theta) ];

% detect near gimbal-lock (cos(theta) ~ 0)
if abs(cos(theta)) < 1e-6
    % fallback: warn and use pseudo-inverse (numerically safer, but still ill-conditioned)
    warning('Gimbal-lock risk: theta ~= ±pi/2. Using pseudo-inverse for Euler kinematics.');
    deuler = pinv(Wn)*omega;
else
    deuler = Wn_inv*omega;
end

% -------------------------
% Rotor geometry (positions in body frame)
% -------------------------
angles = (0:5)'*(2*pi/6);
xi = PARAMS.L*cos(angles); % x positions (6x1)
yi = PARAMS.L*sin(angles); % y positions (6x1)

% Moments from thrust (r_i x F_i), F_i = [0;0;T_i] in body frame
% r_i x F_i = [ y_i * T_i; -x_i * T_i; 0 ]
Mx = sum(yi.*T_i);
My = - sum(xi.*T_i);

% -------------------------
% Rotor reaction torque about z (aerodynamic torque)
% -------------------------
Mz = PARAMS.b*sum(-PARAMS.alpha(:).*u(:));
% (Optional: include time derivative of rotor speeds effect)
% If rotor accelerations (dw) are known, an extra term PARAMS.Im * sum(dw) could be added.

% Total control torque vector in body frame
Tau = [Mx; My; Mz];

% -------------------------
% Gyroscopic torque due to rotor spinning
% -------------------------
w = sqrt(u);
w_net = sum(PARAMS.alpha(:).*w(:)); % net rotor spin (signed)
Gamma = PARAMS.Ir*w_net*[q;-p;0];

% -------------------------
% Rotational dynamics (body frame)
% -------------------------
domega = PARAMS.I\( -cross(omega,PARAMS.I*omega)-Gamma+Tau );

% -------------------------
% Build state derivative
% -------------------------
dx = zeros(12,1);
dx(1:3)   = V;        % position derivatives (velocity in inertial frame)
dx(4:6)   = dV;       % translational acceleration
dx(7:9)   = domega;   % angular acceleration (body rates)
dx(10:12) = deuler;   % Euler angle derivatives

end
