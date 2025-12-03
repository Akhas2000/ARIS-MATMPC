
% Add the path to the functions
addpath(genpath('Functions'));


%% Plot results


% Colors (RGB 0..1)
blue   = [0 0 255]/255;
red    = [220 20 60]/255;
orange = [255 165 0]/255;
green  = [0 205 102]/255;
        



% start generating pictures
switch settings.model
    %% Tracking with soft communication constraints
    case 'GTMR_4_com_soft'


        %% ============================
        %  PLOT RESULTS (NoT vs HoT)
        %  ============================


        % Short-hands from settings
        pA        = settings.pA;
        pR        = settings.pR;
        freq      = settings.freq;
        Power     = settings.Power;
        Bandwidth = settings.Bandwidth;
        beta_B    = settings.beta_B;
        beta_R    = settings.beta_R;
        R_min     = settings.R_min;
        M_ele     = settings.M_ele;
        N_A       = settings.N_A;
        
        %% ------------------------------------------
        %  (Optional) Figure 1: Reference rate field
        %  ------------------------------------------
        if exist('x_vals','var') && exist('y_vals','var') && exist('Rmat','var')
            pathXY = pathXY_ProxyUtility;
        
            figure;
            scatter(x_vals, y_vals, 12, Rmat, 'filled');            % colour = Σ-rate
            axis equal; grid on; colorbar;
            xlabel('x (m)'); ylabel('y (m)');
            hold on;
        
            % BS + users + planned path
            plot(0,0,'rs','MarkerSize',10,'MarkerFaceColor','r');             % BS
            plot(pK(1,:), pK(2,:), 'ro','MarkerSize',10,'MarkerFaceColor','r');% users
            plot(pathXY(:,1), pathXY(:,2), 'r-','LineWidth',2);                % path
            plot(pathXY(1,1), pathXY(1,2), 'rs','MarkerFaceColor','r');        % start
            plot(pathXY(end,1), pathXY(end,2),'rd','MarkerFaceColor','r');     % finish
        
            legend({'Rate samples','BS','Users','Reference path'}, ...
                   'Location','northoutside'); 
            hold off;
        end
        
        %% ------------------------------------------
        %  Figure 2a: 3D Trajectories (NoT vs HoT)
        %  ------------------------------------------
        pUser_ref = [pathXY_ProxyUtility, h_UAV*ones(Ns,1)];  % user/reference path
        
        figure;
        plot3(state_NoT(:,1), state_NoT(:,2), state_NoT(:,3), ...
              'b', 'LineWidth', 2, 'DisplayName', 'NoT Trajectory');
        hold on
        plot3(state_HoT(:,1), state_HoT(:,2), state_HoT(:,3), ...
              'g', 'LineWidth', 2, 'DisplayName', 'HoT Trajectory');
        
        plot3(pUser_ref(:,1), pUser_ref(:,2), pUser_ref(:,3), ...
              'r', 'LineWidth', 2, 'DisplayName', 'Reference Trajectory');
        scatter3(pUser_ref(1,1), pUser_ref(1,2), pUser_ref(1,3), 100,'g^','filled','DisplayName','Start');
        scatter3(pUser_ref(end,1), pUser_ref(end,2), pUser_ref(end,3),100,'b^','filled','DisplayName','Finish');

        % % BS position (origin)
        % scatter3(0,0,0,150,'kp','filled','DisplayName','BS');
        
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
        za = za * 1 + 10.5;  % from 10.5 to 11
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
        
        
        %% ------------------------------------------
        %  Figure 2b: NoT vs HoT — Trajectory components overlay
        %  ------------------------------------------
        
        % --- Build time vectors safely ---
        if exist('time_NoT','var') && numel(time_NoT) >= size(state_NoT,1)
            t_NoT = time_NoT(1:size(state_NoT,1));
        elseif exist('Ts','var')
            t_NoT = (0:size(state_NoT,1)-1) * Ts;
        else
            t_NoT = 0:size(state_NoT,1)-1;
        end
        
        if exist('time_HoT','var') && numel(time_HoT) >= size(state_HoT,1)
            t_HoT = time_HoT(1:size(state_HoT,1));
        elseif exist('Ts','var')
            t_HoT = (0:size(state_HoT,1)-1) * Ts;
        else
            t_HoT = 0:size(state_HoT,1)-1;
        end
        
        % --- Extract components ---
        px_NoT = state_NoT(:,1); py_NoT = state_NoT(:,2); pz_NoT = state_NoT(:,3);
        px_HoT = state_HoT(:,1); py_HoT = state_HoT(:,2); pz_HoT = state_HoT(:,3);
        
        % --- Plot ---
        figure;
        tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
        
        % p_x
        nexttile;
        plot(t_HoT, px_HoT, '-', 'Color', blue, 'LineWidth', 1.5, 'DisplayName', 'HoT: p_x');
        hold on;
        plot(t_NoT, px_NoT, '-.',  'Color', blue, 'LineWidth', 1.5, 'DisplayName', 'NoT: p_x');
        grid on; ylabel('p_x (m)'); legend('show','Location','best');
        
        % p_y
        nexttile;
        plot(t_HoT, py_HoT, '-', 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'HoT: p_y');
        hold on;
        plot(t_NoT, py_NoT, '-.',  'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'NoT: p_y');
        
        grid on; ylabel('p_y (m)'); legend('show','Location','best');
        
        % p_z
        nexttile;
        plot(t_HoT, pz_HoT, '-', 'Color', green, 'LineWidth', 1.5, 'DisplayName', 'HoT: p_z');
        hold on;
        plot(t_NoT, pz_NoT, '-.',  'Color', green, 'LineWidth', 1.5, 'DisplayName', 'NoT: p_z');
        
        
        grid on; ylabel('p_z (m)'); xlabel('Time (s)'); legend('show','Location','best');
        
        % Link time axes for synchronization
        ax = findall(gcf,'Type','axes'); 
        linkaxes(ax,'x');
        xlim([min([t_NoT(:); t_HoT(:)]), max([t_NoT(:); t_HoT(:)])]);
        
        %sgtitle('Evolution of NoT and HoT Trajectories vs Time');

        %% ------------------------------------------
        %  Communication metrics along both flights
        %  ------------------------------------------
        
        % Initial BF using first NoT position
        pU0 = [state_NoT(1,1), state_NoT(1,2), state_NoT(1,3)];
        f_phases_init = array_response_phases_BS(pA, pU0, freq);
        
        % Initial RIS phase diff at identity orientation
        R_fixed = eye(3);
        theta_init = [];
        P_A_rx_RIS0 = phase_array_response_RIS(pR, pU0, [],          R_fixed, freq);
        P_A_tx_RIS0 = phase_array_response_RIS(pR, pU0, p_bar_User,  R_fixed, freq);
        for m_idx = 1:M_ele
            theta_init = [theta_init; P_A_tx_RIS0(m_idx) - P_A_rx_RIS0(m_idx)];
        end
        
        % Storage
        RATES_NoT = [];   % per-user rates (NoT)
        RATES_HoT = [];   % per-user rates (HoT)
        
        Total_NoT_BP = []; Total_HoT_BP = [];
        Total_NoT_B  = []; Total_HoT_B  = [];
        Total_NoT_P  = []; Total_HoT_P  = [];
        
        time_draw = 0;
        iter_draw = 1;
        imax = min(size(state_NoT,1), size(state_HoT,1));
        
        while time_draw(end) < Tf && iter_draw <= imax
        
            % ---------- NoT (free orientation) ----------
            q0 = state_NoT(iter_draw,4); q1 = state_NoT(iter_draw,5);
            q2 = state_NoT(iter_draw,6); q3 = state_NoT(iter_draw,7);
            R_NoT = [ 1-2*(q2^2+q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
                      2*(q1*q2 + q0*q3), 1-2*(q1^2+q3^2),     2*(q2*q3 - q0*q1);
                      2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1),   1-2*(q1^2+q2^2) ];
            pU_NoT = [state_NoT(iter_draw,1), state_NoT(iter_draw,2), state_NoT(iter_draw,3)];
            f_NoT  = array_response_phases_BS(pA, pU_NoT, freq);
        
            theta_NoT = [];
            P_A_rx_RIS = phase_array_response_RIS(pR, pU_NoT, [],          R_NoT, freq);
            P_A_tx_RIS = phase_array_response_RIS(pR, pU_NoT, p_bar_User,  R_NoT, freq);
            for m_idx = 1:M_ele
                theta_NoT = [theta_NoT; P_A_tx_RIS(m_idx) - P_A_rx_RIS(m_idx)];
            end
        
            % ---------- HoT (horizontal orientation tracking) ----------
            q0 = state_HoT(iter_draw,4); q1 = state_HoT(iter_draw,5);
            q2 = state_HoT(iter_draw,6); q3 = state_HoT(iter_draw,7);
            R_HoT = [ 1-2*(q2^2+q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
                      2*(q1*q2 + q0*q3), 1-2*(q1^2+q3^2),     2*(q2*q3 - q0*q1);
                      2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1),   1-2*(q1^2+q2^2) ];
            pU_HoT = [state_HoT(iter_draw,1), state_HoT(iter_draw,2), state_HoT(iter_draw,3)];
            f_HoT  = array_response_phases_BS(pA, pU_HoT, freq);
        
            theta_HoT = [];
            P_A_rx_RIS = phase_array_response_RIS(pR, pU_HoT, [],          R_HoT, freq);
            P_A_tx_RIS = phase_array_response_RIS(pR, pU_HoT, p_bar_User,  R_HoT, freq);
            for m_idx = 1:M_ele
                theta_HoT = [theta_HoT; P_A_tx_RIS(m_idx) - P_A_rx_RIS(m_idx)];
            end
        
            % ---------- Rates ----------
            Rates_NoT_i = Rates_No_Complex_phase(pA,pR,pU_NoT,pK,R_NoT,theta_NoT,f_NoT, Power,Bandwidth,freq,beta_B,beta_R);
            Rates_HoT_i = Rates_No_Complex_phase(pA,pR,pU_HoT,pK,R_HoT,theta_HoT,f_HoT, Power,Bandwidth,freq,beta_B,beta_R);
            RATES_NoT   = [RATES_NoT; Rates_NoT_i];
            RATES_HoT   = [RATES_HoT; Rates_HoT_i];
        
            % Three schemes: BP (both optimized), B (theta frozen), P (BS BF frozen)
            NoT_BP = Rates_No_Complex_phase(pA,pR,pU_NoT,pK,R_NoT,theta_NoT,f_NoT, Power,Bandwidth,freq,beta_B,beta_R);
            HoT_BP = Rates_No_Complex_phase(pA,pR,pU_HoT,pK,R_HoT,theta_HoT,f_HoT, Power,Bandwidth,freq,beta_B,beta_R);
        
            NoT_B  = Rates_No_Complex_phase(pA,pR,pU_NoT,pK,R_NoT,theta_init,f_NoT, Power,Bandwidth,freq,beta_B,beta_R);
            HoT_B  = Rates_No_Complex_phase(pA,pR,pU_HoT,pK,R_HoT,theta_init,f_HoT, Power,Bandwidth,freq,beta_B,beta_R);
        
            NoT_P  = Rates_No_Complex_phase(pA,pR,pU_NoT,pK,R_NoT,theta_NoT,f_phases_init, Power,Bandwidth,freq,beta_B,beta_R);
            HoT_P  = Rates_No_Complex_phase(pA,pR,pU_HoT,pK,R_HoT,theta_HoT,f_phases_init, Power,Bandwidth,freq,beta_B,beta_R);
        
            Total_NoT_BP = [Total_NoT_BP; sum(NoT_BP)];
            Total_HoT_BP = [Total_HoT_BP; sum(HoT_BP)];
            Total_NoT_B  = [Total_NoT_B;  sum(NoT_B)];
            Total_HoT_B  = [Total_HoT_B;  sum(HoT_B)];
            Total_NoT_P  = [Total_NoT_P;  sum(NoT_P)];
            Total_HoT_P  = [Total_HoT_P;  sum(HoT_P)];
        
            % time advance
            nextTime = iter_draw * Ts;
            iter_draw = iter_draw + 1;
            time_draw = [time_draw nextTime];
        end
        
        %% ------------------------------------------
        %  Figure 3: Rates per user (NoT vs HoT)
        %  ------------------------------------------
        kUsers = size(RATES_NoT, 2);
        figure(); hold on;
        for user = 1:kUsers
            plot(time_NoT(2:end), RATES_HoT(:, user), ...
                'LineWidth', 1.5, 'LineStyle', '-', 'DisplayName', ['Rate-HoT ', num2str(user)]);
            plot(time_NoT(2:end), RATES_NoT(:, user), ...
                'LineWidth', 1.5, 'LineStyle', '-.',  'DisplayName', ['Rate-NoT ', num2str(user)]);

        end
        yline(R_min, '--k', 'LineWidth', 2, 'DisplayName', 'R_{min}');
        xlabel('Time (s)'); ylabel('User data rate (bit/s)');
        legend('show', 'Location', 'northoutside', 'NumColumns', 3); grid on; hold off;
        
        %% ------------------------------------------
        %  Figure 4: Total network rate (schemes)
        %  ------------------------------------------
        figure; hold on;

        plot(time_NoT(2:end), Total_HoT_BP, 'b-', 'LineWidth', 1.5, 'DisplayName', 'HoT-BP');
        plot(time_NoT(2:end), Total_NoT_BP, 'b-.',  'LineWidth', 1.5, 'DisplayName', 'NoT-BP');
        
        
        plot(time_NoT(2:end), Total_HoT_P,  'r-', 'LineWidth', 1.5, 'DisplayName', 'HoT-P');
        plot(time_NoT(2:end), Total_NoT_P,  'r-.',  'LineWidth', 1.5, 'DisplayName', 'NoT-P');
        
        
        plot(time_NoT(2:end), Total_HoT_B,  'g-', 'LineWidth', 1.5, 'DisplayName', 'HoT-B');
        plot(time_NoT(2:end), Total_NoT_B,  'g-.',  'LineWidth', 1.5, 'DisplayName', 'NoT-B');
        
        xlabel('Time (s)'); ylabel('Total network rate (bit/s)');
        legend('show','Location','best'); grid on;
        
        %% ------------------------------------------
        %  Figure 5: Per-user transmitted data (cum.)
        %  ------------------------------------------
        TD_NoT = cumsum(RATES_NoT) * Ts;
        TD_HoT = cumsum(RATES_HoT) * Ts;
        
        figure; hold on;
        for user = 1:kUsers
            plot(time_NoT(2:end), TD_HoT(:, user), ...
                'LineWidth',1.5, 'LineStyle','-', 'DisplayName',['Data-HoT ',num2str(user)]);
            plot(time_NoT(2:end), TD_NoT(:, user), ...
                'LineWidth',1.5, 'LineStyle','-.',  'DisplayName',['Data-NoT ',num2str(user)]);

        end
        yline(R_min * Tf, '--k', 'LineWidth', 2, 'DisplayName', 'R_{min}\cdot T_f');
        xlabel('Time (s)'); ylabel('Transmitted data (bit)');
        legend('show', 'Location', 'northoutside', 'NumColumns', 3); grid on; hold off;
        
        %% ------------------------------------------
        %  Figure 6: Cumulative network data (schemes)
        %  ------------------------------------------
        Trans_NoT_BP = cumsum(Total_NoT_BP) * Ts;
        Trans_HoT_BP = cumsum(Total_HoT_BP) * Ts;
        Trans_NoT_B  = cumsum(Total_NoT_B)  * Ts;
        Trans_HoT_B  = cumsum(Total_HoT_B)  * Ts;
        Trans_NoT_P  = cumsum(Total_NoT_P)  * Ts;
        Trans_HoT_P  = cumsum(Total_HoT_P)  * Ts;
        
        figure; hold on;
        plot(time_NoT(2:end), Trans_HoT_BP, 'b-', 'LineWidth',1.5, 'DisplayName','HoT-BP');
        plot(time_NoT(2:end), Trans_NoT_BP, 'b-.',  'LineWidth',1.5, 'DisplayName','NoT-BP');
        
        plot(time_NoT(2:end), Trans_HoT_P,  'r-', 'LineWidth',1.5, 'DisplayName','HoT-P');
        plot(time_NoT(2:end), Trans_NoT_P,  'r-.',  'LineWidth',1.5, 'DisplayName','NoT-P');
        
        plot(time_NoT(2:end), Trans_HoT_B,  'g-', 'LineWidth',1.5, 'DisplayName','HoT-B');
        plot(time_NoT(2:end), Trans_NoT_B,  'g-.',  'LineWidth',1.5, 'DisplayName','NoT-B');
        
        
        xlabel('Time (s)'); ylabel('Cumulative transmitted data (bit)');
        legend('show','Location','best'); grid on;
        
        %% ------------------------------------------
        %  Figure 7: Velocity (HoT run)
        %  ------------------------------------------
        v_HoT  = state_HoT(:,8:10);
        
        v_min = data_HoT.v_min; v_max = data_HoT.v_max;

        vx = v_HoT(:,1);
        vy = v_HoT(:,2);
        vz = v_HoT(:,3);
        
        % Speed over time
        v_mag = sqrt(vx.^2 + vy.^2 + vz.^2);
        
        figure; hold on;
        plot(time_HoT, v_mag, 'LineWidth', 1.5, 'DisplayName', 'V (HoT)');
        %yline(v_min, '--k', 'LineWidth', 1.5, 'HandleVisibility','off');
        yline(v_max, '--k', 'LineWidth', 1.5, 'HandleVisibility','off');
        xlabel('Time (s)'); ylabel('Velocity (m/s)');
        legend('show','Location','best'); ylim([v_min, v_max + 3]); grid on; hold off;
        
        %% ------------------------------------------
        %  Figure 8: Angular velocities (HoT run)
        %  ------------------------------------------
        omega_HoT = state_HoT(:,11:13);
        figure; hold on;
        plot(time_HoT, omega_HoT(:,1), 'LineWidth', 1.5, 'DisplayName', '\omega_x (HoT)');
        plot(time_HoT, omega_HoT(:,2), 'LineWidth', 1.5, 'DisplayName', '\omega_y (HoT)');
        plot(time_HoT, omega_HoT(:,3), 'LineWidth', 1.5, 'DisplayName', '\omega_z (HoT)');
        xlabel('Time (s)'); ylabel('Angular velocity (rad/s)');
        legend('show'); grid on; hold off;
        
        %% ------------------------------------------
        %  Figure 9: Rotor speeds (HoT run)
        %  ------------------------------------------
        Omega_min = data_HoT.Omega_min;
        Omega_max = data_HoT.Omega_max;
        Omega_HoT = controls_HoT;
        
        figure;
        for i = 1:4
            subplot(4,1,i);
            plot(time_HoT, sqrt(Omega_HoT(:,i)), 'LineWidth', 1.25);
            yline(sqrt(Omega_min), '--k', 'LineWidth', 1.5);
            yline(sqrt(Omega_max), '--k', 'LineWidth', 1.5);
            ylim([sqrt(Omega_min) - 10, sqrt(Omega_max) + 10]);
             ylabel(['\Omega_', num2str(i),'HoT (Hz)']); grid on;
        end
        xlabel('Time (s)');

        
        %% ------------------------------------------
        %  Figure 10: Euler angles (HoT run)
        %  ------------------------------------------
        % state_HoT(:,4:7) = [q0 q1 q2 q3] where MATLAB quat2rotm expects [w x y z]
        q_HoT   = state_HoT(:,4:7);
        eul_HoT = zeros(size(q_HoT,1), 3);  % roll pitch yaw (XYZ)
        
        for i = 1:size(q_HoT,1)
            q = q_HoT(i,:);             
            R = quat2rotm(q);                   % Rotation matrix
            eul_HoT(i,:) = rotm2eul(R, 'XYZ');  % Euler angles (XYZ)
        end
        
        roll  = eul_HoT(:,1) * 180/pi;
        pitch = eul_HoT(:,2) * 180/pi;
        yaw   = eul_HoT(:,3) * 180/pi;
        
        figure; hold on;
        plot(time_HoT, roll,  'LineWidth', 1.5, 'DisplayName', 'Roll (HoT)');
        plot(time_HoT, pitch, 'LineWidth', 1.5, 'DisplayName', 'Pitch (HoT)');
        plot(time_HoT, yaw,   'LineWidth', 1.5, 'DisplayName', 'Yaw (HoT)');
        xlabel('Time (s)'); ylabel('Euler angles (deg)');
        legend('show'); grid on; hold off;

        %% ------------------------------------------
        %  Figure 10: Position tracking error(HoT run)
        %  ------------------------------------------
        % Reference states (YOU must provide these)
        
        %% ============================================================
        %          BUILD REFERENCE STATE
        % ============================================================
        state_ref=zeros(Ns-N,13);
        state_ref(:,1:2) = pathXY_ProxyUtility(1:Ns-N,:);  % p_ref(1:2)
        state_ref(:,3)   = h_UAV*ones(Ns-N, 1);            % p_ref(3)
        state_ref(:,4)   = 1*ones(Ns-N, 1);                % q_ref = [1 0 0 0]
        % v_ref, w_ref = 0
        
        p_ref = state_ref(:,1:3);
        q_ref = state_ref(:,4:7);
        v_ref = state_ref(:,8:10);
        w_ref = state_ref(:,11:13);
        
        %% ============================================================
        %          HoT SIGNALS
        % ============================================================
        p_HoT = state_HoT(:,1:3);
        q_HoT = state_HoT(:,4:7);
        v_HoT = state_HoT(:,8:10);
        w_HoT = state_HoT(:,11:13);
        
        %% ============================================================
        %          NoT SIGNALS
        % ============================================================
        p_NoT = state_NoT(:,1:3);
        q_NoT = state_NoT(:,4:7);
        v_NoT = state_NoT(:,8:10);
        w_NoT = state_NoT(:,11:13);
        
        %% ============================================================
        %          ERRORS — HoT
        % ============================================================
        err_p_HoT = p_HoT - p_ref;
        err_v_HoT = v_HoT - v_ref;
        err_w_HoT = w_HoT - w_ref;
        
        err_p_HoT_norm = sqrt(sum(err_p_HoT.^2, 2));
        err_v_HoT_norm = sqrt(sum(err_v_HoT.^2, 2));
        err_w_HoT_norm = sqrt(sum(err_w_HoT.^2, 2));
        
        err_q_HoT = zeros(size(q_HoT,1),1);
        for k = 1:length(err_q_HoT)
            err_q_HoT(k) = quatGeodesicDistance(q_HoT(k,:).', q_ref(k,:).');
        end
        
        %% ============================================================
        %          ERRORS — NoT
        % ============================================================
        err_p_NoT = p_NoT - p_ref;
        err_v_NoT = v_NoT - v_ref;
        err_w_NoT = w_NoT - w_ref;
        
        err_p_NoT_norm = sqrt(sum(err_p_NoT.^2, 2));
        err_v_NoT_norm = sqrt(sum(err_v_NoT.^2, 2));
        err_w_NoT_norm = sqrt(sum(err_w_NoT.^2, 2));
        
        err_q_NoT = zeros(size(q_NoT,1),1);
        for k = 1:length(err_q_NoT)
            err_q_NoT(k) = quatGeodesicDistance(q_NoT(k,:).', q_ref(k,:).');
        end
        
        %% ============================================================
        %          PLOTS (HoT solid — NoT dashed)
        % ============================================================
        figure;
        tiledlayout(4,1,'Padding','compact','TileSpacing','compact');
        
        % --- Position Tracking Error Norm ---
        nexttile;
        plot(time_HoT, err_p_HoT_norm, 'b', 'LineWidth', 1.5, 'DisplayName', 'HoT');
        hold on;
        plot(time_NoT, err_p_NoT_norm, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'NoT');
        grid on; ylabel('‖p - p_d‖ (m)');
        %title('Tracking Errors: HoT (solid) vs NoT (dashed)');
        legend('show');
        
        % --- Velocity Tracking Error Norm ---
        nexttile;
        plot(time_HoT, err_v_HoT_norm, 'b', 'LineWidth', 1.5, 'DisplayName', 'HoT');
        hold on;
        plot(time_NoT, err_v_NoT_norm, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'NoT');
        grid on; ylabel('‖v - v_d‖ (m/s)');
        %legend('show');
        
        % --- Angular Velocity Tracking Error Norm ---
        nexttile;
        plot(time_HoT, err_w_HoT_norm, 'b', 'LineWidth', 1.5, 'DisplayName', 'HoT');
        hold on;
        plot(time_NoT, err_w_NoT_norm, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'NoT');
        grid on; ylabel('‖\omega - \omega_d‖ (rad/s)');
        %legend('show');
        
        % --- Orientation Geodesic Error ---
        nexttile;
        plot(time_HoT, err_q_HoT, 'b', 'LineWidth', 1.5, 'DisplayName', 'HoT');
        hold on;
        plot(time_NoT, err_q_NoT, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'NoT');
        grid on; ylabel('$$\|\log(q \circ q_d^\star)\|$$','Interpreter','latex');
        xlabel('Time (s)');
        %legend('show');




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

