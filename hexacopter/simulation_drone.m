function [t, etats] = simulation_drone()
PARAM.g = 9.81;
PARAM.m = 2;
PARAM.l = 0.25;
PARAM.r = 0.1;
PARAM.h = 0.4;
PARAM.I = [(1/12)*PARAM.m*(3*PARAM.r^2+PARAM.h^2) 0 0; 0 (1/12)*PARAM.m*(3*PARAM.r^2+PARAM.h^2) 0; 0 0 (1/2)*PARAM.m*PARAM.r^2];
PARAM.alfa = [1;-1;1;-1;1;-1];
PARAM.beta = [1;1;1;1;1;1];

x0 = [0;0;0;0;0;0;0;0;0];
%x0 = [0;0;0;0;0;0;0;-pi/3;-pi/3];
%u = [1.5;1.49;1.5;1.49;1.5;1.49];
% u = [1.5;1.5;1.5;1.5;1.5;1.5];
u = [3.7;3.2;3.7;3.2;3.7;3.2];   % empujes (N) -> subirá y girará

tspan = 1:1e-2:2;
f = @(t,x) model_dynamique(t, x, u, PARAM);
[t, etats] = ode45(f, tspan, x0);

end