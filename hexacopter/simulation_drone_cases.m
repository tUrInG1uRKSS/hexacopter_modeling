function [t, etats] = simulation_drone_cases(test_case)

% --- 1. PARÁMETROS BÁSICOS DEL DRON ---
PARAM.g = 9.81;
PARAM.m = 2;         % Masa de 2 kg
PARAM.l = 0.25;      % Longitud del brazo de 0.25 m (25 cm)
PARAM.r = 0.1;       % Radio del cuerpo (cilindro)
PARAM.h = 0.4;       % Altura del cuerpo (cilindro)

% Matriz de Inercia (¡Corregida!)
Ixx = (1/12) * PARAM.m * (3 * PARAM.r^2 + PARAM.h^2);
Iyy = Ixx; % Por simetría
Izz = (1/2) * PARAM.m * PARAM.r^2;
PARAM.I = diag([Ixx, Iyy, Izz]);

% Coeficientes de motor
PARAM.alfa = [1;-1;1;-1;1;-1]; % Dirección de torque de yaw
PARAM.beta = [1;1;1;1;1;1];    % Coeficiente de empuje (asumido 1)
PARAM.kd = 1e-3; % Coef de arrastre para yaw (valor de ejemplo)

% --- 2. CÁLCULOS PARA PRUEBAS ---
% Fuerza de gravedad total
F_g = PARAM.m * PARAM.g; % 2 * 9.81 = 19.62 N
% Empuje necesario para flotar (dividido entre 6 motores)
u_hover = F_g / 6;       % 19.62 / 6 = 3.27 N por motor

% --- 3. SELECCIÓN DE CASO DE PRUEBA ---

% Definir x0 (estado inicial) y u (control) según el caso
% Estado: x = [vx; vy; vz; phi; theta; psi; p; q; r] (9 estados)

switch test_case
    case 0
        % --- PRUEBA DE FLOTACIÓN (HOVER) ---
        x0 = zeros(9, 1);
        u = ones(6, 1) * u_hover; % Empuje exacto para contrarrestar gravedad
        
        % QUÉ DEBERÍA PASAR:
        % El dron debe quedarse perfectamente quieto en [0,0,0].
        % Todos los estados deben permanecer en 0 (salvo errores numéricos).

    case 1
        % --- PRUEBA DE DESPEGUE VERTICAL ---
        x0 = zeros(9, 1);
        u = ones(6, 1) * (u_hover + 0.5); % Un poco más de empuje (3.77 N)
        
        % QUÉ DEBERÍA PASAR:
        % El dron debe acelerar y subir verticalmente (vz > 0).
        % No debe haber rotación (phi, theta, psi = 0) ni movimiento lateral.

    case 2
        % --- PRUEBA DE GIRO (YAW) A LA DERECHA ---
        x0 = zeros(9, 1);
        % Mantenemos empuje total para flotar (3.27*6)
        % Aumentamos motores [1,3,5], reducimos [2,4,6]
        u_yaw = 0.3;
        u = [u_hover + u_yaw;  % Motor 1
             u_hover - u_yaw;  % Motor 2
             u_hover + u_yaw;  % Motor 3
             u_hover - u_yaw;  % Motor 4
             u_hover + u_yaw;  % Motor 5
             u_hover - u_yaw]; % Motor 6
        
        % QUÉ DEBERÍA PASAR:
        % El dron debe flotar (vz ~ 0) pero empezar a girar
        % sobre su eje Z (psi > 0 y r > 0).

    case 3
        % --- PRUEBA DE INCLINACIÓN (PITCH) ADELANTE ---
        x0 = zeros(9, 1);
        % Mantenemos empuje total para flotar
        % (Asumiendo Motor 1 = +X, Motor 4 = -X)
        % Aumentamos empuje atrás (Motor 4), reducimos adelante (Motor 1)
        % Esto crea un torque de cabeceo (My) positivo.
        u_pitch = 0.2;
        u = ones(6, 1) * u_hover;
        u(1) = u_hover - u_pitch; % Menos empuje adelante
        u(4) = u_hover + u_pitch; % Más empuje atrás
        
        % QUÉ DEBERÍA PASAR:
        % El dron debe inclinarse "nariz arriba" (theta > 0 y q > 0)
        % y empezar a moverse hacia atrás (vx < 0).

    case 4
        % --- PRUEBA DE INCLINACIÓN (ROLL) A LA IZQUIERDA ---
        x0 = zeros(9, 1);
        % Mantenemos empuje total para flotar
        % (Asumiendo Motores 2,3 = +Y, Motores 5,6 = -Y)
        % Aumentamos empuje a la derecha (Motores 5,6), reducimos a la izquierda (2,3)
        % Esto crea un torque de alabeo (Mx) negativo.
        u_roll = 0.1;
        u = ones(6, 1) * u_hover;
        u(2) = u_hover - u_roll;
        u(3) = u_hover - u_roll;
        u(5) = u_hover + u_roll;
        u(6) = u_hover + u_roll;
        
        % QUÉ DEBERÍA PASAR:
        % El dron debe inclinarse hacia la izquierda (phi < 0 y p < 0)
        % y empezar a moverse/deslizarse hacia la izquierda (vy > 0).
        % (OJO: vy > 0 puede ser "izquierda" si el eje Y apunta a la izquierda,
        % o "derecha" si apunta a la derecha. ¡Verifica tus ejes!)
        
    case 5
        % --- PRUEBA DE INESTABILIDAD (CAÍDA CON ÁNGULO) ---
        % Empezamos con el dron ya inclinado (0.2 rad ~ 11.5 grados)
        x0 = [0;0;0; 0.2; 0; 0; 0;0;0];
        % Damos solo el empuje para flotar
        u = ones(6, 1) * u_hover;
        
        % QUÉ DEBERÍA PASAR:
        % El empuje (T) apunta 11.5° inclinado. El empuje vertical (T*cos(phi))
        % será MENOR que la gravedad.
        % El dron debe empezar a CAER (vz < 0) y a DESLIZARSE
        % hacia el lado (vy < 0). Demuestra que el sistema es inestable.
        
    case 6
        % --- TU CASO ORIGINAL (Subir y Girar) ---
        x0 = zeros(9, 1);
        u = [3.7; 3.2; 3.7; 3.2; 3.7; 3.2]; 
        
        % QUÉ DEBERÍA PASAR:
        % Empuje total = (3.7+3.2)*3 = 20.7 N.
        % Gravedad = 19.62 N.
        % Como Empuje > Gravedad, el dron debe SUBIR (vz > 0).
        % Como Motores [1,3,5] > [2,4,6], debe GIRAR (Yaw) (psi > 0).
        % ¡Tu comentario original era correcto!

    otherwise
        error("Caso de prueba no válido: %d", test_case);
end

% --- 4. EJECUTAR LA SIMULACIÓN ---
% Rango de tiempo: de 0 a 5 segundos, con pasos de 0.01s
tspan = 0:1e-2:2; 

% Llamada al solver
f = @(t,x) model_dynamique(t, x, u, PARAM);
[t, etats] = ode45(f, tspan, x0);

end