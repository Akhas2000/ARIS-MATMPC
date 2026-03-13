%% Plot results (HoT-only version from FO case)

% Define colors
blue   = [0 0 255]/255;
red    = [220 20 60]/255;
orange = [255 165 0]/255;
green  = [0 205 102]/255;

switch settings.model

    %% ===============================
    %  GTMR_4_com_soft  (Quaternion-based)
    %  ===============================
    case 'GTMR_4_com_soft'

        %% --- 3D Trajectory Plot ----------------------------------------
        pUser_ref = data.REF(1:end,1:3);

        figure;
        plot3(state_sim(:,1),state_sim(:,2),state_sim(:,3), ...
              'b','LineWidth',2,'DisplayName','UAV Trajectory');
        hold on;
        scatter3(state_sim(1,1),state_sim(1,2),state_sim(1,3),100,'go','filled','DisplayName','Start');
        scatter3(state_sim(end,1),state_sim(end,2),state_sim(end,3),100,'bo','filled','DisplayName','End');
        plot3(pUser_ref(:,1),pUser_ref(:,2),pUser_ref(:,3),'r','LineWidth',2,'DisplayName','Reference');
        scatter3(0,0,0,150,'kp','filled','DisplayName','BS position');



                % BS position (origin)
        scatter3(0,0,0,150,'kp','filled','DisplayName','BS');
        
        % Users (first one gets legend, rest hidden)
        num_users = size(pK, 2);
        for kUser = 1:num_users
            if kUser == 1
                scatter3(pK(1,kUser), pK(2,kUser), pK(3,kUser), 100, 'ro','filled', 'DisplayName','Users');
            else
                scatter3(pK(1,kUser), pK(2,kUser), pK(3,kUser), 100, 'ro','filled', 'HandleVisibility','off');
            end
        end
        
        grid on; axis equal
        xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
        legend('show','Location','best');
        
        % ===== (Optional) Stylized BS tower/antenna (purely visual) =====
        scatter3(0,0,0,150,'kp','filled','DisplayName','BS position');          % fixed BS at origin
        
        % Tower: conical trunk (tapered cylinder)
        n_seg = 3;  % number of segments for metallic effect
        [base_x, base_y, z_tower] = cylinder([10, 20], n_seg);  % wide base, narrow top
        z_tower = z_tower * -54;  % height = 54 units downward
        surf(base_x, base_y, z_tower, ...
             'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'k', 'HandleVisibility', 'off');
        
        % Platform: flat small cylinder on top
        [xp, yp, zp] = cylinder(0.7, 12);
        zp = zp * 0.5 + 10;  % placed at height 10
        surf(xp, yp, zp, 'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none', 'HandleVisibility', 'off');
        
        % Antenna: vertical tube
        [xa, ya, za] = cylinder(10, 12);
        za = za * 1 + 10.5;  % from 10.5 to 11.5
        surf(xa, ya, za, 'FaceColor', 'k', 'EdgeColor', 'none', 'HandleVisibility', 'off');
        
        % === Stylized BTS with directional antenna ===
        
        % 1. Conical tower (trunk)
        n_seg = 4;  % 4 sides → square structure
        [pyl_x, pyl_y, pyl_z] = cylinder([5 2], n_seg);  % wide base → narrow top
        pyl_z = pyl_z * 10;
        surf(pyl_x, pyl_y, pyl_z, 'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'k', 'HandleVisibility', 'off');
        
        % 2. Platform on top
        [xp, yp, zp] = cylinder(0.7, 4);  % flat square
        zp = zp * 0.2 + 10;
        surf(xp, yp, zp, 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none', 'HandleVisibility', 'off');
        
        % 3. Antenna box (small cube)
        [x_box, y_box, z_box] = ndgrid([-0.2 0.2], [-0.2 0.2], [0 0.5]);
        x_box = x_box(:); y_box = y_box(:); z_box = z_box(:) + 10.2;
        
        % cube faces
        faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
        patch('Vertices', [x_box y_box z_box], 'Faces', faces, ...
              'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'k', 'HandleVisibility', 'off');
        
        % 4. Directional panel (antenna rectangle)
        panel_width = 0.6; panel_height = 1;
        panel_offset = 0.3;  % distance from tower
        panel_z = 11.2;  % height above the platform
        
        x_panel = [-panel_width/2, panel_width/2, panel_width/2, -panel_width/2];
        y_panel = panel_offset * ones(1, 4);  % offset along Y-axis
        z_panel = panel_z + [0, 0, panel_height, panel_height];
        
        fill3(x_panel, y_panel, z_panel, [1 0.1 0.1], 'FaceAlpha', 1, 'EdgeColor', 'k', 'HandleVisibility', 'off');

        % xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
        % legend('show'); grid on; axis equal;

        %% --- Communication Metric Initialization -----------------------
        pA        = settings.pA;
        pR        = settings.pR;
        freq      = settings.freq;
        Power     = settings.Power;
        Bandwidth = settings.Bandwidth;
        beta_B    = settings.beta_B;
        beta_R    = settings.beta_R;
        R_min     = settings.R_min;
        M_ele     = settings.M_ele;
        K_User=settings.K_User;
        R   = eye(3);

        Ts = Ts; Tf = Tf;
        if ~exist('time','var'), time = (0:size(state_sim,1)-1)*Ts; end

        % Initial beamforming vector
        pU0 = state_sim(1,1:3);
        f_phases_init = array_response_phases_BS(pA, pU0, freq);

        % Initial RIS phase shift (at R)
        P_A_rx_RIS0 = phase_array_response_RIS(pR,pU0,[],R,freq);
        P_A_tx_RIS0 = phase_array_response_RIS(pR,pU0,p_bar_User,R,freq);
        theta_init  = P_A_tx_RIS0 - P_A_rx_RIS0;

        % Storage variables
        RATES_HoT = [];
        Total_HoT_BP = [];
        Total_HoT_B  = [];
        Total_HoT_P  = [];

        iter_draw = 1;
        time_draw = 0;

        %% --- Main Loop -------------------------------------------------
        while time_draw(end) < Tf && iter_draw < size(state_sim,1)

            % Rotation Matrix (from quaternion)
            q0 = state_sim(iter_draw,4); q1 = state_sim(iter_draw,5);
            q2 = state_sim(iter_draw,6); q3 = state_sim(iter_draw,7);
            R  = [ 1-2*(q2^2+q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
                   2*(q1*q2 + q0*q3), 1-2*(q1^2+q3^2),     2*(q2*q3 - q0*q1);
                   2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1),   1-2*(q1^2+q2^2) ];

            % UAV position
            pU = state_sim(iter_draw,1:3);

            % Beamforming vector
            f_phases = array_response_phases_BS(pA, pU, freq);

            % RIS phase optimization (horizontal orientation)
            P_A_rx_RIS = phase_array_response_RIS(pR, pU, [],         R, freq);
            P_A_tx_RIS = phase_array_response_RIS(pR, pU, p_bar_User, R, freq);
            theta_HoT  = P_A_tx_RIS - P_A_rx_RIS;

            % Compute rates (3 schemes)
            Rates_HoT_BP = Rates_No_Complex_phase(pA,pR,pU,pK,R,theta_HoT,f_phases,      Power,Bandwidth,freq,beta_B,beta_R);
            Rates_HoT_B  = Rates_No_Complex_phase(pA,pR,pU,pK,R,theta_init,f_phases,     Power,Bandwidth,freq,beta_B,beta_R);
            Rates_HoT_P  = Rates_No_Complex_phase(pA,pR,pU,pK,R,theta_HoT,f_phases_init, Power,Bandwidth,freq,beta_B,beta_R);

            RATES_HoT = [RATES_HoT; Rates_HoT_BP];
            Total_HoT_BP = [Total_HoT_BP; sum(Rates_HoT_BP)];
            Total_HoT_B  = [Total_HoT_B;  sum(Rates_HoT_B)];
            Total_HoT_P  = [Total_HoT_P;  sum(Rates_HoT_P)];

            % Next iteration
            iter_draw = iter_draw + 1;
            time_draw = [time_draw (iter_draw-1)*Ts];
        end

        %% --- Time Vector Alignment -------------------------------------
        L = size(RATES_HoT,1);
        t_plot = time(2:L+1);
        kUsers = size(RATES_HoT,2);

        %% --- Figure: User Rates ----------------------------------------
        figure(); hold on;
        for user = 1:kUsers
            plot(t_plot, RATES_HoT(:,user), 'LineWidth',1.5,'DisplayName',['Rate-HoT u',num2str(user)]);
        end
        yline(R_min, '--k', 'LineWidth', 2, 'DisplayName', 'R_{min}');
        xlabel('Time (s)'); ylabel('User Data Rate (bit/s)');
        legend('show','Location','best'); grid on; hold off;

        %% --- Figure: Total Network Rate --------------------------------
        figure; hold on;
        plot(t_plot, Total_HoT_BP, 'b-', 'LineWidth',1.5,'DisplayName','HoT-BP');
        plot(t_plot, Total_HoT_B,  'g-', 'LineWidth',1.5,'DisplayName','HoT-B');
        plot(t_plot, Total_HoT_P,  'r-', 'LineWidth',1.5,'DisplayName','HoT-P');
        xlabel('Time (s)'); ylabel('Total Network Rate (bit/s)');
        legend('show','Location','best'); grid on;

        %% --- Figure: Cumulative User Data -------------------------------
        TD_HoT = cumsum(RATES_HoT)*Ts;
        figure; hold on;
        for user = 1:kUsers
            plot(t_plot, TD_HoT(:,user), 'LineWidth',1.5,'DisplayName',['Data-HoT u',num2str(user)]);
        end
        yline(R_min*Tf, '--k', 'LineWidth',2,'DisplayName','R_{min}·T_f');
        xlabel('Time (s)'); ylabel('Transmitted Data (bit)');
        legend('show','Location','best'); grid on; hold off;

        %% --- Figure: Cumulative Network Data ----------------------------
        Trans_HoT_BP = cumsum(Total_HoT_BP)*Ts;
        Trans_HoT_B  = cumsum(Total_HoT_B)*Ts;
        Trans_HoT_P  = cumsum(Total_HoT_P)*Ts;
        figure; hold on;
        plot(t_plot, Trans_HoT_BP,'b','LineWidth',1.5,'DisplayName','HoT-BP');
        plot(t_plot, Trans_HoT_B, 'g','LineWidth',1.5,'DisplayName','HoT-B');
        plot(t_plot, Trans_HoT_P, 'r','LineWidth',1.5,'DisplayName','HoT-P');
        xlabel('Time (s)'); ylabel('Cumulative Transmitted Data (bit)');
        legend('show','Location','best'); grid on;

        %% --- Figure: Velocity ------------------------------------------
        v_sim = state_sim(:,8:10);
        v_min = data.v_min; v_max = data.v_max;

        vx = v_sim(:,1);
        vy = v_sim(:,2);
        vz = v_sim(:,3);
        
        % Speed over time
        v_mag = sqrt(vx.^2 + vy.^2 + vz.^2);

        figure; hold on;
        plot(time, v_mag, 'LineWidth',1.5,'DisplayName','V');
        % plot(time, vx, 'LineWidth',1.5,'DisplayName','Vx');
        % plot(time, vy, 'LineWidth',1.5,'DisplayName','Vy');
        % plot(time, vz, 'LineWidth',1.5,'DisplayName','Vy');
        yline(v_min,'--k','LineWidth',1.5); yline(v_max,'--k','LineWidth',1.5);
        xlabel('Time (s)'); ylabel('Velocity (m/s)');
        legend('show'); grid on; hold off;



        %% --- Figure: Angular Velocities --------------------------------
        omega_sim = state_sim(:,11:13);
        figure; hold on;
        plot(time, omega_sim(:,1),'LineWidth',1.5,'DisplayName','\omega_x');
        plot(time, omega_sim(:,2),'LineWidth',1.5,'DisplayName','\omega_y');
        plot(time, omega_sim(:,3),'LineWidth',1.5,'DisplayName','\omega_z');
        xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
        legend('show'); grid on; hold off;

        %% --- Figure: Rotor Speeds --------------------------------------
        % Omega_min = data.Omega_min; Omega_max = data.Omega_max;
        % Omega_sim = controls_MPC;
        % figure;
        % for i = 1:4
        %     subplot(4,1,i);
        %     plot(time, Omega_sim(:,i),'LineWidth',1);
        %     yline(Omega_min,'--k','LineWidth',1);
        %     yline(Omega_max,'--k','LineWidth',1);
        %     ylim([Omega_min-100, Omega_max+100]);
        %     title(['\Omega_',num2str(i)]); ylabel('HoT (Hz^2)'); grid on;
        % end
        % xlabel('Time (s)');



        Omega_min = data.Omega_min;
        Omega_max = data.Omega_max;
        Omega_sim = controls_MPC;
        
        figure;
        for i = 1:4
            subplot(4,1,i);
            plot(time, sqrt(Omega_sim(:,i)), 'LineWidth', 1.25);
            yline(sqrt(Omega_min), '--k', 'LineWidth', 1.5);
            yline(sqrt(Omega_max), '--k', 'LineWidth', 1.5);
            ylim([sqrt(Omega_min) - 10, sqrt(Omega_max) + 10]);
             ylabel(['\Omega_', num2str(i),'HoT (Hz)']); grid on;
        end
        xlabel('Time (s)');

        %% --- Figure: Orientation (Euler) -------------------------------
        q_sim = state_sim(:,4:7);
        eul_sim = zeros(size(q_sim,1),3);
        for i = 1:size(q_sim,1)
            q = q_sim(i,:);
            Rq = quat2rotm(q);
            eul_sim(i,:) = rotm2eul(Rq,'XYZ');
        end
        roll  = eul_sim(:,1)*180/pi;
        pitch = eul_sim(:,2)*180/pi;
        yaw   = eul_sim(:,3)*180/pi;
        figure; hold on;
        plot(time, roll,'LineWidth',1.5,'DisplayName','Roll (HoT)');
        plot(time, pitch,'LineWidth',1.5,'DisplayName','Pitch (HoT)');
        plot(time, yaw,'LineWidth',1.5,'DisplayName','Yaw (HoT)');
        xlabel('Time (s)'); ylabel('Euler Angles (deg)');
        legend('show'); grid on; hold off;
    










        %% ============================================================
        %      OBJECTIVE DECOMPOSITION 
        % ============================================================
        
        Q_vec = data.q_0;      % weight vector used in model
        Q_diag = diag(Q_vec);
        
        % Extract signals
        p     = state_sim(:,1:3);
        v     = state_sim(:,8:10);
        sv    = state_sim(:,14:13+K_User);
        omega = state_sim(:,11:13);
        q= state_sim(:,4:7);
        q_ref=zeros(size(q,1),4);
        q_ref(:,1)=1;

        err_q = zeros(size(q,1),1);
        for k = 1:length(err_q)
            err_q(k) = quatGeodesicDistance(q(k,:).', q_ref(k,:).');
        end
        eta   = err_q;   % geodesic distance already computed
        



        % References
        p_ref=zeros(size(p,1),3);
        p_ref(:,1:2) = pathXY_ProxyUtility(1:size(p,1),:);  % p_ref(1:2)
        p_ref(:,3)   = h_UAV*ones(size(p,1), 1);            % p_ref(3)

        v_ref = zeros(size(v));
        sv_ref = zeros(size(sv));
        omega_ref = zeros(size(omega));
        eta_ref = zeros(size(eta));
        
        % Errors
        e_p     = p - p_ref;
        e_v     = v - v_ref;
        e_sv    = sv - sv_ref;
        e_omega = omega - omega_ref;
        e_eta   = eta - eta_ref;
        
        % Compute contributions
        J_p     = 0.5 * sum((e_p.^2)     .* Q_vec(1:3)', 2);
        J_v     = 0.5 * sum((e_v.^2)     .* Q_vec(4:6)', 2);
        J_sv    = 0.5 * sum((e_sv.^2)    .* Q_vec(7:6+K_User)', 2);
        J_omega = 0.5 * sum((e_omega.^2) .* Q_vec(7+K_User:9+K_User)', 2);
        J_eta   = 0.5 * (e_eta.^2)       .* Q_vec(end);
        
        % Total cost
        J_total = J_p + J_v + J_sv + J_omega + J_eta;



        %% ============================================================
        %      OBJECTIVE DECOMPOSITION — SUBPLOTS (HoT)
        % ============================================================
        
        figure;
        tiledlayout(6,1,'Padding','compact','TileSpacing','compact');
        
        % ------------------------------------------------------------
        nexttile;
        plot(time, J_p, 'b','LineWidth',1.5);
        grid on;
        ylabel('J_p');
        title('Position Contribution');
        
        % ------------------------------------------------------------
        nexttile;
        plot(time, J_v, 'Color',[0.85 0.33 0.1],'LineWidth',1.5);
        grid on;
        ylabel('J_v');
        title('Velocity Contribution');
        
        % ------------------------------------------------------------
        nexttile;
        plot(time, J_sv, 'm','LineWidth',1.5);
        grid on;
        ylabel('J_{sv}');
        title('Slack (QoS) Contribution');
        
        % ------------------------------------------------------------
        nexttile;
        plot(time, J_omega, 'g','LineWidth',1.5);
        grid on;
        ylabel('J_\omega');
        title('Angular Velocity Contribution');
        
        % ------------------------------------------------------------
        nexttile;
        plot(time, J_eta, 'k','LineWidth',1.5);
        grid on;
        ylabel('J_\eta');
        title('Orientation (Geodesic) Contribution');
        
        % ------------------------------------------------------------
        nexttile;
        plot(time, J_total, 'LineWidth',2);
        grid on;
        ylabel('J_{total}');
        xlabel('Time (s)');
        title('Total Objective');
        
        % Link all x-axes
        ax = findall(gcf,'Type','axes');
        linkaxes(ax,'x');

    
    %% ===============================
    %  Simplified_UAV_Kinematic (Yaw-only)
    %  ===============================
    case 'Simplified_UAV_Kinematic'

        %% --- Similar computation, yaw-only rotation matrix -------------
        % You can adapt same structure with R = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1]
        % and same plots, naming them HoT.
        % (Omitted here for brevity — can expand if you want.)
        
end



%% Functions 

function d = quatGeodesicDistance(q1, q2)
% Geodesic distance between two UNIT quaternions q1 and q2
% q = [w x y z]' (scalar-first convention)
% Assumes q1 and q2 are already normalized

    % Conjugate of q2
    q2_conj = [q2(1); -q2(2:4)];

    % Quaternion multiplication: q = q1 * q2*
    q = quatMultiply(q1, q2_conj);

    % Numerical clamp for safety
    w = min(max(q(1), -1), 1);

    % Geodesic distance = rotation angle between them
    angle = acos(w);
    d = 2 * angle;
end

%-------------------------------------
% Quaternion multiplication
%-------------------------------------
function q = quatMultiply(q1, q2)
    w1 = q1(1); v1 = q1(2:4);
    w2 = q2(1); v2 = q2(2:4);

    w = w1*w2 - dot(v1, v2);
    v = w1*v2 + w2*v1 + cross(v1, v2);

    q = [w; v];
end
