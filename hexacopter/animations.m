function animations(positions, angles)
% ANIMATIONS  Animación 3D simple de un hexacóptero
%   animations(positions, angles)
%     positions : Nx3 (x,y,z) en m
%     angles    : Nx3 (phi,theta,psi) en rad

% Validaciones básicas
if nargin < 2
    error('Se requieren positions y angles (Nx3 cada una).');
end
if size(positions,2) ~= 3 || size(angles,2) ~= 3
    error('positions y angles deben ser Nx3.');
end
N = size(positions,1);
if size(angles,1) ~= N
    error('positions y angles deben tener el mismo número de filas (misma duración).');
end

% Parámetros visuales (ajustables)
L = 0.3;             % brazo desde el centro hasta cada rotor (m)
rotor_radius = 0.125; % radio visual del rotor (m)
body_marker_size = 30;
circle_n = 18;       % resolución de los discos de rotores
arm_width = 2;       % grosor líneas brazos
axis_scale = 0.25;   % longitud flechas de los ejes del cuerpo

% Precalcular posiciones de rotores en marco del cuerpo
angles_rot = (0:5) * 2*pi/6; % 6 rotores
rb = [L*cos(angles_rot); L*sin(angles_rot); zeros(1,6)]; % 3x6

% Preparar figura
fh = figure('Name','Animación Hexacóptero','Color','w','NumberTitle','off');
ax = axes('Parent',fh);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45,25);

% --- Cálculo robusto de límites (reemplaza uso de range)
pos_max = max(positions,[],1);
pos_min = min(positions,[],1);
ranges = pos_max - pos_min;              % [rx ry rz]
pad = max(0.5, 0.2 * max(ranges));       % margen de visualización

xmin = pos_min(1)-pad; xmax = pos_max(1)+pad;
ymin = pos_min(2)-pad; ymax = pos_max(2)+pad;
zmin = pos_min(3)-pad; zmax = pos_max(3)+pad;
if zmin <= 0, zmin = min(zmin, -0.5); end
axis(ax,[xmin xmax ymin ymax zmin zmax]);

% Dibujar trayectoria (trayecto completo como línea tenue)
traj_h = plot3(ax, positions(:,1), positions(:,2), positions(:,3), ':', 'LineWidth', 0.8, 'Color',[0.6 0.6 0.6]);

% Crear objetos gráficos iniciales
body_h = scatter3(ax, positions(1,1), positions(1,2), positions(1,3), body_marker_size, 'filled', 'MarkerFaceColor',[0 0.4470 0.7410]);

% Brazos: 6 line objects (cada brazo desde centro al rotor)
arm_h = gobjects(6,1);
for i=1:6
    arm_h(i) = plot3(ax, [0 0], [0 0], [0 0], '-', 'LineWidth', arm_width);
end

% Rotores: cada uno como línea cerrada que forma un disco
rotor_h = gobjects(6,1);
theta_c = linspace(0,2*pi,circle_n);
circ_xy = [cos(theta_c); sin(theta_c); zeros(1,circle_n)]; % círculo en plano xy del cuerpo
for i=1:6
    rotor_h(i) = plot3(ax, zeros(1,circle_n), zeros(1,circle_n), zeros(1,circle_n), '-', 'LineWidth', 1.5);
end

% Flechas de ejes del cuerpo (X:rojo, Y:verde, Z:azul)
qx = quiver3(ax,0,0,0,0,0,0,'r','LineWidth',1.5,'MaxHeadSize',0.5);
qy = quiver3(ax,0,0,0,0,0,0,'g','LineWidth',1.5,'MaxHeadSize',0.5);
qz = quiver3(ax,0,0,0,0,0,0,'b','LineWidth',1.5,'MaxHeadSize',0.5);

% Texto de tiempo / info
txt = title(ax, sprintf('Frame 1 / %d', N));

% Loop de animación (muestreo adaptativo para timelapses largos)
max_frames = 400; % objetivo de frames para mantener fluidez
step = max(1, round(N/max_frames));

for k = 1:step:N
    pos = positions(k,:).';        % 3x1
    phi = angles(k,1); theta = angles(k,2); psi = angles(k,3);
    R = rotz(psi) * roty(theta) * rotx(phi); % cuerpo->inercial
    
    % actualizar cuerpo (centro)
    set(body_h, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
    
    % actualizar brazos y rotores
    for i=1:6
        rb_i = rb(:,i);                     % en cuerpo
        r_in = (R * rb_i) + pos;           % en inercial
        % brazo: centro -> rotor
        set(arm_h(i), 'XData', [pos(1) r_in(1)], 'YData', [pos(2) r_in(2)], 'ZData', [pos(3) r_in(3)]);
        % rotor disco (girar el disco en el plano local del cuerpo)
        circ_pts = rotor_radius * circ_xy;   % círculo en marco cuerpo centrado en origen
        circ_world = R * circ_pts + r_in;   % trasladar al rotor global
        set(rotor_h(i), 'XData', circ_world(1,:), 'YData', circ_world(2,:), 'ZData', circ_world(3,:));
    end
    
    % actualizar ejes de cuerpo (flechas)
    Xb = R * [axis_scale;0;0] + pos;
    Yb = R * [0;axis_scale;0] + pos;
    Zb = R * [0;0;axis_scale] + pos;
    set(qx, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), 'UData', Xb(1)-pos(1), 'VData', Xb(2)-pos(2), 'WData', Xb(3)-pos(3));
    set(qy, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), 'UData', Yb(1)-pos(1), 'VData', Yb(2)-pos(2), 'WData', Yb(3)-pos(3));
    set(qz, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), 'UData', Zb(1)-pos(1), 'VData', Zb(2)-pos(2), 'WData', Zb(3)-pos(3));
    
    % actualizar título / tiempo
    set(txt, 'String', sprintf('Frame %d / %d — x=%.2f y=%.2f z=%.2f', k, N, pos(1), pos(2), pos(3)));
    
    drawnow limitrate
    % si cierras la figura, terminar el loop
    if ~ishandle(fh), return; end
end

end
