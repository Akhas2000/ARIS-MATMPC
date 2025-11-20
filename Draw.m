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
        figure; hold on;
        plot(time, v_sim(:,1), 'LineWidth',1.5,'DisplayName','v_x');
        plot(time, v_sim(:,2), 'LineWidth',1.5,'DisplayName','v_y');
        plot(time, v_sim(:,3), 'LineWidth',1.5,'DisplayName','v_z');
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
        Omega_min = data.Omega_min; Omega_max = data.Omega_max;
        Omega_sim = controls_MPC;
        figure;
        for i = 1:4
            subplot(4,1,i);
            plot(time, Omega_sim(:,i),'LineWidth',1);
            yline(Omega_min,'--k','LineWidth',1);
            yline(Omega_max,'--k','LineWidth',1);
            ylim([Omega_min-100, Omega_max+100]);
            title(['\Omega_',num2str(i)]); ylabel('HoT (Hz^2)'); grid on;
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

    %% ===============================
    %  Simplified_UAV_Kinematic (Yaw-only)
    %  ===============================
    case 'Simplified_UAV_Kinematic'

        %% --- Similar computation, yaw-only rotation matrix -------------
        % You can adapt same structure with R = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1]
        % and same plots, naming them HoT.
        % (Omitted here for brevity — can expand if you want.)
        
end
