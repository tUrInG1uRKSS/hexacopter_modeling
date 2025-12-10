function params = get_hex_params()
%GET_HEX_PARAMS  Parámetros del hexacóptero
%   Devuelve una estructura con masa, inercia, geometría y constantes del modelo

% Parámetros físicos
params.m = 1.5;         % masa [kg]
params.g = 9.81;        % gravedad [m/s^2]
params.l = 0.25;        % longitud del brazo [m]

% Constantes de empuje y par
params.kf = 6.11e-8 * 1e7;   % coef. de empuje [N/(rad/s)^2]
params.km = 1.5e-9 * 1e8;    % coef. de par [Nm/(rad/s)^2]

% Matriz de inercia (aproximada, simétrica)
params.I = diag([0.02, 0.02, 0.04]); % [kg*m^2]

% Posición de los rotores en el plano XY (hexágono regular)
theta = (0:5) * pi/3; % 0°, 60°, 120°, 180°, 240°, 300°
params.rpos = params.l * [cos(theta); sin(theta); zeros(1,6)];

% Sentido de giro de cada rotor (+1 horario, -1 antihorario)
params.sgn = [1 -1 1 -1 1 -1];

end
