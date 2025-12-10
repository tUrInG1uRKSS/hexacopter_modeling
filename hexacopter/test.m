clear all
close all
clc

% Case 0: El dron debe quedarse perfectamente quieto en [0,0,0].
% Case 1: El dron debe acelerar y subir verticalmente (vz > 0).
% Case 2: El dron debe flotar (vz ~ 0) pero empezar a girar.
% Case 3: El dron debe inclinarse "nariz arriba" (theta > 0 y q > 0) y empezar a moverse hacia atrás (vx < 0).
% Case 4: El dron debe inclinarse hacia la izquierda (phi < 0 y p < 0) y empezar a moverse/deslizarse hacia la izquierda (vy > 0).
% Case 5: El dron debe empezar a CAER (vz < 0) y a DESLIZARSE hacia el lado (vy < 0). Demuestra que el sistema es inestable.
% Case 6: Como Empuje > Gravedad, el dron debe SUBIR (vz > 0). Como Motores [1,3,5] > [2,4,6], debe GIRAR (Yaw) (psi > 0).

test_case = 5;
[t, etats] = simulation_drone_cases(test_case);
V = etats(:,1:3);
angles = etats(:,4:6);
positions = cumtrapz(t, V);
animations(positions, angles);