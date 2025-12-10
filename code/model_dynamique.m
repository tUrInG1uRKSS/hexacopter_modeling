function xdot = model_dynamique(~, x, u, PARAMS)
% model_dynamique  Dinámica simplificada de un hexacóptero
% Entradas:
%   ~     : tiempo (no usado, pero requerido por ODE45)
%   x     : estado (9x1) = [vx; vy; vz; phi; theta; psi; p; q; r]
%   u     : entradas de los 6 rotores (6x1), empujes en N
%   PARAMS : estructura con campos .g, .m, .l, .I, .alfa (6x1), .beta (6x1)
%
% Salida:
%   xdot  : derivada del estado (9x1)

% Extraer estados
vx = x(4); vy = x(5); vz = x(6);
omega = x(7:9);    % [p; q; r]
phi = x(10); theta = x(11); psi = x(12);

% Posiciones de los rotores en el cuerpo (distribuidos uniformemente)
angles = (0:5)' * (2*pi/6);       % 0, 60, 120, ...
xi = PARAMS.L * cos(angles);      % x_i
yi = PARAMS.L * sin(angles);      % y_i

% Fuerzas (empujes) por rotor (si el usuario pasa otra cosa, aquí se interpreta como N)
F = PARAMS.beta .* u(:);          % asegurar columna 6x1
T_total = sum(F);                % empuje total (N) en eje z del cuerpo

% Momentos producidos por empujes (r x F_body) con F_body = [0;0;F_i]
% r_i = [x_i; y_i; 0] => r_i x F_i = [ y_i*F_i; -x_i*F_i; 0 ]
Mx = sum( yi .* F );
My = - sum( xi .* F );

% Momento de guiñada por arrastre/rotación de rotores (proporcional a u)
Mz = sum( -PARAMS.alfa .* u(:) ) * PARAMS.kd;

% Matriz rotación (cuerpo -> inercial) ZYX
R = rotz(psi) * roty(theta) * rotx(phi);

% Dinámica traslacional (velocidades están en marco inercial)
vdot = (1/PARAMS.m) * ( R * [0;0;T_total] ) - [0;0;PARAMS.g];

% Cinemática Euler (relación entre p,q,r y phi_dot,theta_dot,psi_dot)
p = omega(1); q = omega(2); r = omega(3);
T_e = [ 1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
        0, cos(phi),           -sin(phi);
        0, sin(phi)/cos(theta), cos(phi)/cos(theta) ];
euler_dot = T_e * omega;

% Dinámica rotacional
Tau = [Mx; My; Mz];
omega_dot = PARAMS.I \ ( Tau - cross(omega, PARAMS.I * omega) );

% Construir xdot (mismo orden que x)
xdot = [ vdot(:); euler_dot(:); omega_dot(:) ];

end

