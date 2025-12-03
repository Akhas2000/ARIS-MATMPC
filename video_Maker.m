% === UAV Flight Video Simulation ===

% === Assumptions ===
% Required variables already in workspace:
% - state_sim [N x ≥13] (x, y, z, q0, q1, q2, q3, ...)
% - pK         [3 x K]   (user positions)
% - pUser_ref  [N x 3]   (reference trajectory)
% - Tf         [scalar]  (desired video duration in seconds)

%% Put NoT for the NoT scheme and HoT for the HoT one
state_sim=state_NoT;
pUser_ref = [pathXY_ProxyUtility, h_UAV*ones(Ns,1)];  % user/reference path

%% === SETTINGS ===
frameRate = 15;                       
video_filename = 'UAV_Flight_Trajectory_NoT.avi';

%% === Compute Frame Sampling Step ===
N = size(state_sim, 1);
total_frames = Tf * frameRate;
step = max(1, floor(N / total_frames));  

%% === VIDEO SETUP ===
v = VideoWriter(video_filename);
v.FrameRate = frameRate;
v.Quality = 100;
open(v);

fig = figure('Position', [100 100 1920 1080]);
set(fig, 'PaperPositionMode', 'auto');  

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
grid on; axis equal; view(3); hold on;


view(30, 35);          % <-- set custom 3D perspective (azimuth, elevation)
camlight headlight;    % <-- add lighting from the camera
lighting gouraud;      % <-- smooth lighting
axis vis3d;            % <-- lock aspect ratio to prevent camera distortion
hold on;


%% === FIXED AXIS LIMITS ===

x_min = -200;
x_max = 200;
y_min = -200;
y_max = 200;
z_min = -69;
z_max = 140;

xlim([x_min, x_max]);
ylim([y_min, y_max]);
zlim([z_min, z_max]);

%% === MAIN LOOP ===
for i = 1:step:N
    cla; hold on;

    % --- Base Station (Stylized BTS) ---
    n_seg = 3;
    [base_x, base_y, z_tower] = cylinder([10, 20], n_seg);
    z_tower = z_tower * -54;
    surf(base_x, base_y, z_tower, 'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'k');

    [xp, yp, zp] = cylinder(0.7, 12);
    zp = zp * 0.5 + 10;
    surf(xp, yp, zp, 'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none');

    [xa, ya, za] = cylinder(10, 12);
    za = za * 1 + 10.5;
    surf(xa, ya, za, 'FaceColor', 'k', 'EdgeColor', 'none');

    n_seg = 4;
    [pyl_x, pyl_y, pyl_z] = cylinder([5 2], n_seg);
    pyl_z = pyl_z * 10;
    surf(pyl_x, pyl_y, pyl_z, 'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'k');

    [xp, yp, zp] = cylinder(0.7, 4);
    zp = zp * 0.2 + 10;
    surf(xp, yp, zp, 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none');

    [x_box, y_box, z_box] = ndgrid([-0.2 0.2], [-0.2 0.2], [0 0.5]);
    x_box = x_box(:); y_box = y_box(:); z_box = z_box(:) + 10.2;
    faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
    patch('Vertices', [x_box y_box z_box], 'Faces', faces, ...
          'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'k');

    panel_width = 0.6; panel_height = 1; panel_offset = 0.3; panel_z = 11.2;
    x_panel = [-panel_width/2, panel_width/2, panel_width/2, -panel_width/2];
    y_panel = panel_offset * ones(1, 4);
    z_panel = panel_z + [0, 0, panel_height, panel_height];
    fill3(x_panel, y_panel, z_panel, [1 0.1 0.1], 'FaceAlpha', 1, 'EdgeColor', 'k');

    % --- Users ---
    for k = 1:size(pK, 2)
        scatter3(pK(1,k), pK(2,k), pK(3,k), 300, 'ro', 'filled');
    end

    % --- Reference Trajectory ---
    plot3(pUser_ref(:,1), pUser_ref(:,2), pUser_ref(:,3), ...
          'r-', 'LineWidth', 2);

    % --- UAV Trajectory Trail ---
    plot3(state_sim(1:i,1), state_sim(1:i,2), state_sim(1:i,3), ...
          'b-', 'LineWidth', 2);

    % --- UAV Pose ---
    pos = state_sim(i,1:3);
    q   = state_sim(i,4:7);
    R   = quat2rotm(q);
    drawQuadrotor_Large(pos, R);

    drawnow;
    frame = getframe(fig);
    writeVideo(v, frame);
    
end

close(v);
disp(['✅ Video saved to ', video_filename]);

%% === UAV DRAWING FUNCTION ===
function drawQuadrotor_Large(p, R)
    hold on;
    L = 10;
    green = [0 205 102]/255;
    red   = [220 20 60]/255;

    rotors = [ L  L 0;
              -L  L 0;
              -L -L 0;
               L -L 0]';
    world_rotors = R * rotors + p(:);

    plot3([world_rotors(1,1), world_rotors(1,3)], ...
          [world_rotors(2,1), world_rotors(2,3)], ...
          [world_rotors(3,1), world_rotors(3,3)], 'k-', 'LineWidth', 4);

    plot3([world_rotors(1,2), world_rotors(1,4)], ...
          [world_rotors(2,2), world_rotors(2,4)], ...
          [world_rotors(3,2), world_rotors(3,4)], 'k-', 'LineWidth', 4);

    for i = 1:4
        rotor_color = (i <= 2)*green + (i > 2)*red;
        scatter3(world_rotors(1,i), world_rotors(2,i), world_rotors(3,i), ...
                 300, rotor_color, 'filled');
    end

    scatter3(p(1), p(2), p(3), 200, 'k', 'filled');

    %% === RIS Grid ===
    ris_size = 30; offset_z = -10; M = 6; N = 6;
    [Xg, Yg] = meshgrid(linspace(-ris_size/2, ris_size/2, N), ...
                        linspace(-ris_size/2, ris_size/2, M));
    Zg = offset_z * ones(size(Xg));
    ris_body_points = [Xg(:)'; Yg(:)'; Zg(:)'];
    ris_world_points = R * ris_body_points + p(:);
    scatter3(ris_world_points(1,:), ris_world_points(2,:), ris_world_points(3,:), ...
             40, [0.2 0.5 1], 's', 'filled');
end
