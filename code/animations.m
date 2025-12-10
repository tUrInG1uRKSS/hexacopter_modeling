function animations(positions, data, PARAMS, type)
% ANIMATIONS: 3D animation of a hexacopter using EULER angles or QUATERNIONS
%
%   animations(positions, data, PARAMS, type)
%
%   positions : Nx3 matrix of positions over time
%   data      : Nx3 (Euler angles) or Nx4 (Quaternions)
%   PARAMS    : Struct containing geometric parameters (L, l, etc.)
%   type      : 'euler' or 'quat'

%% -------- INPUT VALIDATION --------
if nargin < 4
    error('Usage: animations(positions, data, PARAMS, type)');
end

if size(positions,2) ~= 3
    error('positions must be an Nx3 matrix');
end

N = size(positions,1);

switch lower(type)
    case 'euler'
        if size(data,2) ~= 3
            error('With type="euler", data must be Nx3 (phi, theta, psi)');
        end
    case 'quat'
        if size(data,2) ~= 4
            error('With type="quat", data must be Nx4 (q0 q1 q2 q3)');
        end
    otherwise
        error('type must be "euler" or "quat"');
end

%% -------- VISUAL PARAMETERS --------
L = PARAMS.L;
rotor_radius = PARAMS.l/2;
body_marker_size = 30;
circle_n = 18;
arm_width = 2;
axis_scale = 0.25;

% rotor positions in body frame
angles_rot = (0:5) * 2*pi/6;
rb = [L*cos(angles_rot); 
      L*sin(angles_rot); 
      zeros(1,6)];

%% -------- FIGURE --------
fig_title = sprintf('Hexacopter Animation (%s attitude)', upper(type));

fh = figure('Name',fig_title,'Color','w','NumberTitle','off');
ax = axes('Parent',fh);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
view(45,25);

% automatic axis padding
pos_max = max(positions,[],1);
pos_min = min(positions,[],1);
pad = max(0.5, 0.2*max(pos_max-pos_min));
axis(ax, [pos_min(1)-pad, pos_max(1)+pad, ...
          pos_min(2)-pad, pos_max(2)+pad, ...
          pos_min(3)-pad, pos_max(3)+pad]);

% trajectory
plot3(ax, positions(:,1), positions(:,2), positions(:,3), ...
      ':','Color',[.6 .6 .6]);

% body marker
body_h = scatter3(ax, positions(1,1), positions(1,2), positions(1,3), ...
                  body_marker_size, 'filled', ...
                  'MarkerFaceColor',[0 0.4470 0.7410]);

% arms
arm_h = gobjects(6,1);
for i=1:6
    arm_h(i) = plot3(ax, [0 0], [0 0], [0 0], '-', ...
                     'LineWidth', arm_width, 'Color','k');
end

% rotors
rotor_h = gobjects(6,1);
theta_c = linspace(0,2*pi,circle_n);
circ_xy = [cos(theta_c); sin(theta_c); zeros(1,circle_n)];

for i=1:6
    rotor_h(i) = plot3(ax, zeros(1,circle_n), zeros(1,circle_n), ...
                       zeros(1,circle_n), '-', 'LineWidth', 1.5);
end

% body axes
qx = quiver3(ax,0,0,0,0,0,0,'r','LineWidth',1.5);
qy = quiver3(ax,0,0,0,0,0,0,'g','LineWidth',1.5);
qz = quiver3(ax,0,0,0,0,0,0,'b','LineWidth',1.5);

% title text
txt = title(ax, sprintf('Frame 1 / %d', N));

%% -------- ANIMATION LOOP --------
max_frames = 400;
step = max(1, round(N/max_frames));

for k = 1:step:N
    pos = positions(k,:).';

    % --------------------------
    % ROTATION MATRIX SELECTION
    % --------------------------
    if strcmpi(type,'euler')
        phi = data(k,1);
        theta = data(k,2);
        psi = data(k,3);
        R = rotz(psi) * roty(theta) * rotx(phi);

    elseif strcmpi(type,'quat')
        q = data(k,:).';
        q = q / norm(q);   % normalize quaternion
        R = quatToRotm(q);
    end

    %% Update body position
    set(body_h, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));

    %% Update arms and rotors
    for i=1:6
        rb_i = rb(:,i);
        r_in = R*rb_i + pos;

        set(arm_h(i), ...
            'XData', [pos(1), r_in(1)], ...
            'YData', [pos(2), r_in(2)], ...
            'ZData', [pos(3), r_in(3)]);

        circ_pts = rotor_radius * circ_xy;
        circ_world = R*circ_pts + r_in;

        set(rotor_h(i), ...
            'XData', circ_world(1,:), ...
            'YData', circ_world(2,:), ...
            'ZData', circ_world(3,:));
    end

    %% Update body axes
    Xb = R*[axis_scale;0;0] + pos;
    Yb = R*[0;axis_scale;0] + pos;
    Zb = R*[0;0;axis_scale] + pos;

    set(qx,'XData',pos(1),'YData',pos(2),'ZData',pos(3), ...
           'UData',Xb(1)-pos(1),'VData',Xb(2)-pos(2),'WData',Xb(3)-pos(3));

    set(qy,'XData',pos(1),'YData',pos(2),'ZData',pos(3), ...
           'UData',Yb(1)-pos(1),'VData',Yb(2)-pos(2),'WData',Yb(3)-pos(3));

    set(qz,'XData',pos(1),'YData',pos(2),'ZData',pos(3), ...
           'UData',Zb(1)-pos(1),'VData',Zb(2)-pos(2),'WData',Zb(3)-pos(3));

    %% Update animation title
    set(txt,'String', sprintf('Frame %d / %d', k, N));

    drawnow limitrate

    if ~ishandle(fh)
        return;
    end
end

end
