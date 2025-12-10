function dx = dynamics_model_quat(~, x, u, PARAMS)
% dynamics_model_euler: Dynamics of a hexacopter (Euler ZYX convention)
% Inputs:
%   ~      : time (not used here, required by ODE solvers)
%   x      : state-vector (13x1) = [px; py; pz; vx; vy; vz; p; q; r; q0; q1; q2; q3]
%            (Positions, velocities, body angular rates, quaternion)
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
%   dx : derivative of state-vector (13x1)

% -------------------------
% Defaults / safety checks
% -------------------------
if ~isfield(PARAMS,'beta'), PARAMS.beta = ones(6,1); end
if ~isfield(PARAMS,'alpha'), PARAMS.alpha = [+1; -1; +1; -1; +1; -1]; end

% -------------------------
% Extract states
% -------------------------
px = x(1); py = x(2); pz = x(3);
vx = x(4); vy = x(5); vz = x(6);
omega = x(7:9);
quat = x(10:13)/norm(x(10:13));

% -------------------------
% Rotation matrix body -> inertial from unit quaternion
% -------------------------
Q = quatToRotm(quat);

% -------------------------
% Thrust forces
% -------------------------
u = max(u, 0);
T_i = PARAMS.k*(PARAMS.beta(:).*u(:)); % thrust per rotor (N)

% -------------------------
% Translational dynamics (in inertial frame)
% -------------------------
V = [vx; vy; vz];
dV = [0;0;-PARAMS.g] + (1/PARAMS.m)*(Q*[0;0;sum(T_i)]);

% -------------------------
% Quaternion kinematics
% dq = 0.5 * Omega(omega) * q
% Omega(omega) defined below
% -------------------------
p = omega(1); q = omega(2); r = omega(3);
OMEGA = [  0, -p, -q, -r;
           p,  0,  r, -q;
           q, -r,  0,  p;
           r,  q, -p,  0 ];
dq = 0.5 * OMEGA * quat;

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
dx = zeros(13,1);
dx(1:3)   = V;        % position derivatives (velocity in inertial frame)
dx(4:6)   = dV;       % translational acceleration
dx(7:9)   = domega;   % angular acceleration (body rates)
dx(10:13) = dq;   % Euler angle derivatives

end