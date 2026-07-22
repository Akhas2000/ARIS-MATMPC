% Add the path to the functions
addpath(genpath('Functions'));

%% ============================
%  GLOBAL STYLE SETTINGS (for LaTeX-ready figures)
%  Figures are exported at their final print size (3.5 in wide,
%  single-column) so fonts/lines do NOT get shrunk when included
%  in LaTeX. Tune FIGSIZE if your column width differs.
% ============================
% FS      = 20;                % axis / tick label font size
% FSL     = 16;                 % legend font size
% LW      = 2.5;                % standard line width
% LWth    = 3.5;                 % thicker/emphasis line width
% MS      = 9;                  % standard marker size
% MS_big  = 12;                  % larger marker size (scatter highlights)
% AXLW    = 1.5;                 % axis box line width
% FIGSIZE = [0 0 3.5 2.8];        % figure physical size in inches (match \includegraphics width)


FS      = 10;                % axis / tick label font size
FSL     = 3;                 % legend font size
LW      = 1.5;                % standard line width
LWth    = 1.5;                 % thicker/emphasis line width
MS      = 6;                  % standard marker size
MS_big  = 6;                  % larger marker size (scatter highlights)
AXLW    = 0.5;                 % axis box line width
% FIGSIZE = [0 0 3.5 2.8];        % figure physical size in inches (match \includegraphics width)



                             %   breathing room without shrinking the plot area
% 
% FS      = 10;               % axis / tick label font size (MATLAB default)
% FSL     = 9;                 % legend font size (MATLAB default, slightly below axis)
% LW      = 0.5;               % standard line width (MATLAB default)
% LWth    = 0.5;                % MATLAB has no separate "thick" default — same as LW
% MS      = 6;                 % standard marker size (MATLAB default)
% MS_big  = 6;                  % MATLAB has no separate "big" default — same as MS
% AXLW    = 0.5;                % axis box line width (MATLAB default)
% FIGSIZE = [0 0 8 6];           % MATLAB default figure size in inches (560x420 px screen size ≈ this)


%% Plot results


% Colors (RGB 0..1)
blue   = [0 0 255]/255;
red    = [220 20 60]/255;
orange = [255 165 0]/255;
green  = [0 205 102]/255;

% Colors for additional benchmarks
purple = [148 0 211]/255;  % SL (Straight Line)
teal   = [0 128 128]/255;  % HOV (Hovering UAV)
        

% start generating pictures
switch settings.model
    %% Tracking with soft communication constraints
    case 'GTMR_4_com_soft'


        %% ============================
        %  PLOT RESULTS (NoT vs HoT)
        %  ============================

        Tf=Tf_init;
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
        K_User    = settings.K_User;
        
        % Check if state_SL exists; if not, fallback to empty to avoid errors
        if ~exist('state_SL','var')
            state_SL = [];
        end

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
            p_bs = plot(0, 0,'rs','MarkerSize', MS_big,'MarkerFaceColor','r');             % BS
            p_us = plot(pK(1,:), pK(2,:), 'ro','MarkerSize', MS,'MarkerFaceColor','r');% users
            p_pt = plot(pathXY(:,1), pathXY(:,2), 'r-','LineWidth', LWth);                % path
            plot(pathXY(1,1), pathXY(1,2), 'rs','MarkerFaceColor','r','HandleVisibility', 'off');        % start
            plot(pathXY(end,1), pathXY(end,2),'rd','MarkerFaceColor','r','HandleVisibility', 'off');     % finish
            
            % Add SL Path and HOV Position to 2D Plot
            if ~isempty(state_SL)
                p_sl = plot(state_SL(:,1), state_SL(:,2), '--', 'Color', purple, 'LineWidth', LWth);
            end
            p_hov = plot(pU_opt_HOV(1), pU_opt_HOV(2), 'k*', 'MarkerSize', MS_big, 'LineWidth', LW);
            
            if ~isempty(state_SL)
                legend({'Rate samples','BS','Users','Reference path', 'SL Path', 'HOV Position'}, ...
                       'Location','northoutside'); 
            else
                legend({'Rate samples','BS','Users','Reference path', 'HOV Position'}, ...
                       'Location','northoutside'); 
            end
            % --- Match reference PDF axis limits ---
            lim_xmin = xmin; lim_xmax = xmax;
            lim_ymin = ymin; lim_ymax = ymax;
            xlim([lim_xmin lim_xmax]);
            ylim([lim_ymin lim_ymax]);
            cb = colorbar;
            cb.Label.String = 'Total network rate (bit/s)';
            cb.Label.FontSize = FSL;

            

            % Set physical figure size close to your intended print size (single column ≈ 3.5in wide)
            %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);
            
            % Bump up font sizes BEFORE export so they're readable at final print size
            set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
            set(gca, 'FontSize', FS, 'LineWidth', AXLW);



            hold off;


        end
        
        %% ------------------------------------------
        %  Figure 2a: 3D Trajectories (NoT vs HoT vs SL vs HOV)
        %  ------------------------------------------
        pUser_ref = [pathXY_ProxyUtility, h_UAV*ones(Ns,1)];  % user/reference path
        
        figure;
        plot3(state_NoT(:,1), state_NoT(:,2), state_NoT(:,3), ...
              'b', 'LineWidth', LWth, 'DisplayName', 'NoT Trajectory');
        hold on
        plot3(state_HoT(:,1), state_HoT(:,2), state_HoT(:,3), ...
              'g', 'LineWidth', LWth, 'DisplayName', 'HoT Trajectory');
              
        % if ~isempty(state_SL)
        %     plot3(state_SL(:,1), state_SL(:,2), state_SL(:,3), ...
        %           '-', 'Color', purple, 'LineWidth', LWth, 'DisplayName', 'SL Trajectory');
        % end
        
        % HOV Position (Black Star)
       % scatter3(pU_opt_HOV(1), pU_opt_HOV(2), pU_opt_HOV(3), 210,'kp', 'filled', 'DisplayName', 'HOV Position');
        
        plot3(pUser_ref(:,1), pUser_ref(:,2), pUser_ref(:,3), ...
              'r', 'LineWidth', LWth, 'DisplayName', 'Reference Trajectory');
        scatter3(pUser_ref(1,1), pUser_ref(1,2), pUser_ref(1,3), 140,'g^','filled','DisplayName','Start');
        scatter3(pUser_ref(end,1), pUser_ref(end,2), pUser_ref(end,3), 140,'b^','filled','DisplayName','Finish');

        % Users (first one gets legend, rest hidden)
        num_users = size(pK, 2);
        for kUser = 1:num_users
            if kUser == 1
                scatter3(pK(1,kUser), pK(2,kUser), pK(3,kUser), 70,'ro','filled', 'DisplayName','Users');
            else
                scatter3(pK(1,kUser), pK(2,kUser), pK(3,kUser), 70,'ro','filled', 'HandleVisibility','off');
            end
        end
        
        grid on; axis equal
        xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
        legend('show','Location','best');
        
 
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
        
        fill3(x_panel, y_panel, z_panel, [1 0.1 0.1], 'FaceAlpha', 1,'EdgeColor', 'k', 'HandleVisibility', 'off');
        % ===== (Optional) Stylized BS tower/antenna (purely visual) =====
        scatter3(0,0,12, 210,'rs','filled','DisplayName','BS position');          % fixed BS at origin



        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);

       
        
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
        tiledlayout(3, 1,'TileSpacing','compact','Padding','compact');
        
        % p_x
        nexttile;
        plot(t_HoT, px_HoT, '-', 'Color', blue, 'LineWidth', LW, 'DisplayName', 'HoT: p_x');
        hold on;
        plot(t_NoT, px_NoT, '--',  'Color', red, 'LineWidth', LW, 'DisplayName', 'NoT: p_x');
        grid on; ylabel('p_x (m)'); legend('show','Location','best');
        
        % p_y
        nexttile;
        plot(t_HoT, py_HoT, '-', 'Color', blue, 'LineWidth', LW, 'DisplayName', 'HoT: p_y');
        hold on;
        plot(t_NoT, py_NoT, '--',  'Color', red, 'LineWidth', LW, 'DisplayName', 'NoT: p_y');
        
        grid on; ylabel('p_y (m)'); legend('show','Location','best');
        
        % p_z
        nexttile;
        plot(t_HoT, pz_HoT, '-', 'Color', blue, 'LineWidth', LW, 'DisplayName', 'HoT: p_z');
        hold on;
        plot(t_NoT, pz_NoT, '--',  'Color', red, 'LineWidth', LW, 'DisplayName', 'NoT: p_z');
        
        
        grid on; ylabel('p_z (m)'); xlabel('Time (s)'); legend('show','Location','best');
        
        % Link time axes for synchronization
        ax = findall(gcf,'Type','axes'); 
        linkaxes(ax,'x');
        xlim([min([t_NoT(:); t_HoT(:)]), max([t_NoT(:); t_HoT(:)])]);

        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);

        %% ------------------------------------------
        %  Communication metrics along all flights
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
        RATES_NoT = []; Total_NoT_BP = []; Total_NoT_B = []; Total_NoT_P = [];
        RATES_HoT = []; Total_HoT_BP = []; Total_HoT_B = []; Total_HoT_P = [];
        RATES_SL  = []; Total_SL_BP  = [];
        RATES_HOV = []; Total_HOV_BP = [];
        
        time_draw = 0;
        iter_draw = 1;
        
        % Ensure loop bound accommodates all available models safely
        imax = min(size(state_NoT,1), size(state_HoT,1));
        if ~isempty(state_SL)
            imax = min(imax, size(state_SL,1));
        end
        
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
            
            % ---------- SL (Straight Line BP only) ----------
            if ~isempty(state_SL)
                q0 = state_SL(iter_draw,4); q1 = state_SL(iter_draw,5);
                q2 = state_SL(iter_draw,6); q3 = state_SL(iter_draw,7);
                R_SL = [ 1-2*(q2^2+q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
                         2*(q1*q2 + q0*q3), 1-2*(q1^2+q3^2),     2*(q2*q3 - q0*q1);
                         2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1),   1-2*(q1^2+q2^2) ];
                pU_SL = [state_SL(iter_draw,1), state_SL(iter_draw,2), state_SL(iter_draw,3)];
                f_SL  = array_response_phases_BS(pA, pU_SL, freq);
        
                theta_SL = [];
                P_A_rx_RIS = phase_array_response_RIS(pR, pU_SL, [],          R_SL, freq);
                P_A_tx_RIS = phase_array_response_RIS(pR, pU_SL, p_bar_User,  R_SL, freq);
                for m_idx = 1:M_ele
                    theta_SL = [theta_SL; P_A_tx_RIS(m_idx) - P_A_rx_RIS(m_idx)];
                end
                Rates_SL_i = Rates_No_Complex_phase(pA,pR,pU_SL,pK,R_SL,theta_SL,f_SL, Power,Bandwidth,freq,beta_B,beta_R);
                RATES_SL   = [RATES_SL; Rates_SL_i];
                Total_SL_BP = [Total_SL_BP; sum(Rates_SL_i)];
            end
            
            % ---------- HOV (Hovering UAV BP only) ----------
            R_HOV = eye(3);
            pU_HOV = [pU_opt_HOV(1), pU_opt_HOV(2), pU_opt_HOV(3)];
            f_HOV  = f_opt_HOV;
            
            
            theta_HOV = theta_opt_HOV;

            Rates_HOV_i = Rates_No_Complex_phase(pA,pR,pU_HOV,pK,R_HOV,theta_HOV,f_HOV, Power,Bandwidth,freq,beta_B,beta_R);
            RATES_HOV   = [RATES_HOV; Rates_HOV_i];
            Total_HOV_BP = [Total_HOV_BP; sum(Rates_HOV_i)];
        
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
        
        % Secure uniform time vector matching the truncated simulation steps
        t_plot = time_draw(2:end);
        t_plot = t_plot(:); 
        
        figure(); hold on;
        for user = 1:kUsers
            plot(t_plot, RATES_HoT(:, user), ...
                'LineWidth', LW, 'LineStyle', '-', 'DisplayName', ['Rate-HoT ', num2str(user)]);
            plot(t_plot, RATES_NoT(:, user), ...
                'LineWidth', LW, 'LineStyle', '-.',  'DisplayName', ['Rate-NoT ', num2str(user)]);

        end
        yline(R_min, '--k', 'LineWidth', LWth, 'DisplayName', 'R_{min}');
        xlabel('Time (s)'); ylabel('User data rate (bit/s)');
        legend('show', 'Location', 'northoutside', 'NumColumns', 3); grid on; hold off;

        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);


        %% ------------------------------------------
        %  Figure 4: Average User Rate (Mean only)
        %  ------------------------------------------
        
        % Statistics (mean only)
        mean_HoT = mean(RATES_HoT, 2);
        mean_NoT = mean(RATES_NoT, 2);
        
        mean_HoT = mean_HoT(:);
        mean_NoT = mean_NoT(:);
        
        figure; 
        hold on; 
        box on;
        
        % =========================
        % HoT
        % =========================
        h1 = plot(t_plot, mean_HoT, ...
         'Color', blue, ...
         'LineWidth', LWth);
        
        % =========================
        % NoT
        % =========================
        h2 = plot(t_plot, mean_NoT, ...
         'Color', orange, ...
         'LineWidth', LW);

        % =========================
        % SL
        % =========================
        h_sl = [];
        if ~isempty(RATES_SL)
            mean_SL = mean(RATES_SL, 2);
            h_sl = plot(t_plot, mean_SL(:), ...
             'Color', purple, ...
             'LineWidth', LW, ...
             'LineStyle', '-');
        end
        
        % =========================
        % HOV
        % =========================
        mean_HOV = mean(RATES_HOV, 2);
        h_hov = plot(t_plot, mean_HOV(:), ...
         'Color', teal, ...
         'LineWidth', LW, ...
         'LineStyle', '-');

        % =========================
        % HoT (Re-plot so it stays on top)
        % =========================
        h1_prime = plot(t_plot, mean_HoT, ...
         'Color', blue, ...
         'LineWidth', LW);
         
        % =========================
        % QoS line
        % =========================
        h3 = yline(R_min,'--k','LineWidth', LWth);
        
        xlabel('Time (s)','FontSize', FS);
        ylabel('Average user rate (bit/s)','FontSize', FS);
        
        grid on;
        
        leg_handles = [h1 h2];
        leg_labels = {'HoT (Average)', 'NoT (Average)'};
        
        if ~isempty(h_sl)
            leg_handles(end+1) = h_sl;
            leg_labels{end+1} = 'SL (Average)';
        end
        leg_handles(end+1) = h_hov;
        leg_labels{end+1} = 'HOV (Average)';
        
        leg_handles(end+1) = h3;
        leg_labels{end+1} = 'R_{min}';
        
        legend(leg_handles, leg_labels, 'Location','northoutside','NumColumns',3);


        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        
        hold off;

        %% ------------------------------------------
        %  Figure 5: Total network rate (schemes)
        %  ------------------------------------------
        figure; hold on;

        plot(t_plot, Total_HoT_BP, 'b-', 'LineWidth', LW, 'DisplayName', 'HoT-BP');
        plot(t_plot, Total_NoT_BP, 'b--',  'LineWidth', LW, 'DisplayName', 'NoT-BP');
        
        plot(t_plot, Total_HoT_P,  'r-', 'LineWidth', LW, 'DisplayName', 'HoT-P');
        plot(t_plot, Total_NoT_P,  'r--',  'LineWidth', LW, 'DisplayName', 'NoT-P');
        
        plot(t_plot, Total_SL_BP,  '-','color', purple, 'LineWidth', LW, 'DisplayName', 'SL');

        plot(t_plot, Total_HoT_B,  '-','color', "#FF8800", 'LineWidth', LW, 'DisplayName', 'HoT-B');
        plot(t_plot, Total_NoT_B,  '--','color', "#FF8800",  'LineWidth', LW, 'DisplayName', 'NoT-B');

        
        
        xlabel('Time (s)'); ylabel('Total network rate (bit/s)');
        legend('show','Location','best'); grid on;

        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        
        %% ------------------------------------------
        %  Figure 6: Per-user transmitted data (cum.)
        %  ------------------------------------------
        TD_NoT = cumsum(RATES_NoT) * Ts;
        TD_HoT = cumsum(RATES_HoT) * Ts;
        
        figure; hold on;
        for user = 1:kUsers
            plot(t_plot, TD_HoT(:, user), ...
                'LineWidth', LW, 'LineStyle','-', 'DisplayName',['Data-HoT ',num2str(user)]);
            plot(t_plot, TD_NoT(:, user), ...
                'LineWidth', LW, 'LineStyle','--',  'DisplayName',['Data-NoT ',num2str(user)]);

        end
        yline(R_min * Tf, '--k', 'LineWidth', LWth, 'DisplayName', 'R_{min}\cdot T_f');
        xlabel('Time (s)'); ylabel('Transmitted data (bit)');
        legend('show', 'Location', 'northoutside', 'NumColumns', 3); grid on; 
        
        
        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        
        hold off;

        %% Compute statistics across users
        %% ------------------------------------------
        % Average Cumulative Transmitted Data (Mean ± Std)
        %% ------------------------------------------
        
        % Compute cumulative transmitted data
        TD_NoT = cumsum(RATES_NoT) * Ts;
        TD_HoT = cumsum(RATES_HoT) * Ts;
        TD_SL  = cumsum(RATES_SL)  * Ts;
        
        % Statistics across users
        mean_HoT = mean(TD_HoT,2);  std_HoT = std(TD_HoT,0,2);
        mean_NoT = mean(TD_NoT,2);  std_NoT = std(TD_NoT,0,2);
        mean_SL  = mean(TD_SL,2);   std_SL  = std(TD_SL,0,2);
        
        figure;
        hold on;
        box on;
        
        % =========================
        % Shaded std bands (draw least important first, most important last)
        % =========================
        hShadeSL = fill([t_plot; flipud(t_plot)], ...
            [mean_SL-std_SL; flipud(mean_SL+std_SL)], ...
            purple, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
        
        hShadeNoT = fill([t_plot; flipud(t_plot)], ...
            [mean_NoT-std_NoT; flipud(mean_NoT+std_NoT)], ...
            red, 'FaceAlpha', 0.15, 'EdgeColor', 'none');
        
        hShadeHoT = fill([t_plot; flipud(t_plot)], ...
            [mean_HoT-std_HoT; flipud(mean_HoT+std_HoT)], ...
            blue, 'FaceAlpha', 0.22, 'EdgeColor', 'none');
        
        % =========================
        % Mean curves — distinct linestyle + linewidth per condition
        % =========================
        hMeanSL  = plot(t_plot, mean_SL,  'Color', purple, 'LineWidth', LW, 'LineStyle', ':');
        hMeanNoT = plot(t_plot, mean_NoT, 'Color', red,    'LineWidth', LW, 'LineStyle', '--');
        hMeanHoT = plot(t_plot, mean_HoT, 'Color', blue,   'LineWidth', LWth, 'LineStyle', '-');
        
        % =========================
        % Sparse markers to reinforce line identity
        % =========================
        markStep = max(1, round(length(t_plot)/15));   % ~15 markers across the curve
        markIdx  = 1:markStep:length(t_plot);
        
        plot(t_plot(markIdx), mean_SL(markIdx),  's', 'Color', purple, ...
             'MarkerFaceColor', purple, 'MarkerSize', MS, 'HandleVisibility','off');
        plot(t_plot(markIdx), mean_NoT(markIdx), '^', 'Color', red, ...
             'MarkerFaceColor', red, 'MarkerSize', MS, 'HandleVisibility','off');
        plot(t_plot(markIdx), mean_HoT(markIdx), 'o', 'Color', blue, ...
             'MarkerFaceColor', blue, 'MarkerSize', MS, 'HandleVisibility','off');
        
        % =========================
        % QoS threshold
        % =========================
        hThr = yline(R_min*Tf, '--k', 'LineWidth', LWth);
        
        % =========================
        % Labels & formatting
        % =========================
        xlabel('Time (s)', 'FontSize', FS);
        ylabel('Average transmitted data (bit)', 'FontSize', FS);
        grid on;
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        ytickformat('%.1f');
        
        % =========================
        % Legend
        % =========================
        legend([hMeanHoT, hMeanNoT, hMeanSL, ...
                hShadeHoT, hShadeNoT, hShadeSL, hThr], ...
               {'Mean HoT', 'Mean NoT', 'Mean SL', ...
                'HoT \pm1 std', 'NoT \pm1 std', 'SL \pm1 std', ...
                'R_{min}T_f'}, ...
               'Location', 'northoutside', 'NumColumns', 6,'FontSize', FS);
        
        % =========================
        % Figure size for LaTeX export
        % =========================
        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        
        hold off;
        
        %% ------------------------------------------
        %  Figure 7: Cumulative network data (schemes)
        %  ------------------------------------------
        Trans_NoT_BP = cumsum(Total_NoT_BP) * Ts;
        Trans_HoT_BP = cumsum(Total_HoT_BP) * Ts;
        Trans_NoT_B  = cumsum(Total_NoT_B)  * Ts;
        Trans_HoT_B  = cumsum(Total_HoT_B)  * Ts;
        Trans_NoT_P  = cumsum(Total_NoT_P)  * Ts;
        Trans_HoT_P  = cumsum(Total_HoT_P)  * Ts;
        
        figure; hold on;
        Trans_HOV_BP = cumsum(Total_HOV_BP) * Ts;
        plot(t_plot, Trans_HOV_BP, '-', 'Color', teal, 'LineWidth', LW, 'DisplayName', 'HOV-BP');

        plot(t_plot, Trans_HoT_BP, 'b-', 'LineWidth', LW, 'DisplayName','HoT-BP');
        plot(t_plot, Trans_NoT_BP, 'b--',  'LineWidth', LW, 'DisplayName','NoT-BP');
        


        
        plot(t_plot, Trans_HoT_P,  'r-', 'LineWidth', LW, 'DisplayName','HoT-P');
        plot(t_plot, Trans_NoT_P,  'r--',  'LineWidth', LW, 'DisplayName','NoT-P');

        if exist('Total_SL_BP','var') && ~isempty(Total_SL_BP)
            Trans_SL_BP = cumsum(Total_SL_BP) * Ts;
            plot(t_plot, Trans_SL_BP, '-', 'Color', purple, 'LineWidth', LW, 'DisplayName', 'SL-BP');
        end
        
        
        plot(t_plot, Trans_HoT_B,  '-' ,'color', "#FF8800",'LineWidth', LW, 'DisplayName','HoT-B');
        plot(t_plot, Trans_NoT_B,  '--', 'color', "#FF8800", 'LineWidth', LW, 'DisplayName','NoT-B');
        
        xlabel('Time (s)'); ylabel('Cumulative transmitted data (bit)');
        legend('show','Location','best'); grid on;

        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        
        %% ------------------------------------------
        %  Figure 8: Velocity (HoT run)
        %  ------------------------------------------
        v_HoT  = state_HoT(:,8:10);
        
        v_min = data_HoT.v_min; v_max = data_HoT.v_max;

        vx = v_HoT(:,1);
        vy = v_HoT(:,2);
        vz = v_HoT(:,3);
        
        % Speed over time
        v_mag = sqrt(vx.^2 + vy.^2 + vz.^2);
        
        figure; hold on;
        plot(time_HoT, v_mag, 'LineWidth', LW );
        yline(v_max, '--r', 'LineWidth', LW);
        xlabel('Time (s)'); ylabel('Velocity (m/s)');
        ylim([v_min, v_max + 3]); grid on; hold off;

        lgd = legend('v (HoT)','$\bar{v}$','Interpreter','latex');
        lgd.FontSize = FSL;   

        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        
        %% ------------------------------------------
        %  Figure 9-a: Angular velocities (HoT run)
        %  ------------------------------------------
        omega_HoT = state_HoT(:,11:13);
        figure; hold on;
        plot(time_HoT, omega_HoT(:,1), 'LineWidth', LW, 'DisplayName', '\omega_x (HoT)');
        plot(time_HoT, omega_HoT(:,2), 'LineWidth', LW, 'DisplayName', '\omega_y (HoT)');
        plot(time_HoT, omega_HoT(:,3), 'LineWidth', LW, 'DisplayName', '\omega_z (HoT)');
        xlabel('Time (s)'); ylabel('Angular velocity (rad/s)');
        legend('show'); grid on; 
        
        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        
        
        hold off;

        %% ------------------------------------------
        %  Figure 9-b: Angular velocities (HoT run)
        %  with zoomed steady-state inset
        %% ------------------------------------------
        
        omega_HoT = state_HoT(:,11:13);
        
        figure;
        
        %% ==========================
        % Main axes
        %% ==========================
        ax1 = axes;
        hold(ax1,'on');
        box(ax1,'on');
        
        plot(ax1,time_HoT,omega_HoT(:,1),...
            'LineWidth',LW,...
            'DisplayName','\omega_x (HoT)');
        
        plot(ax1,time_HoT,omega_HoT(:,2),...
            'LineWidth',LW,...
            'DisplayName','\omega_y (HoT)');
        
        plot(ax1,time_HoT,omega_HoT(:,3),...
            'LineWidth',LW,...
            'DisplayName','\omega_z (HoT)');
        
        xlabel('Time (s)');
        ylabel('Angular velocity (rad/s)');
        
        grid(ax1,'on');
        
        legend(ax1,'show','Location','southwest');
        
        set(ax1,...
            'FontSize',FS,...
            'LineWidth',AXLW);
        
        %% ==========================
        % Zoom region
        %% ==========================
        
        xZoom = [0 4];
        yZoom = [-11 2];
        
        rectangle(ax1,...
            'Position',[xZoom(1),yZoom(1),...
                        diff(xZoom),diff(yZoom)],...
            'EdgeColor','k',...
            'LineStyle','--',...
            'LineWidth',1);
        
        %% ==========================
        % Inset
        %% ==========================
        
        ax2 = axes(...
            'Position',[0.60 0.27 0.30 0.28],...
            'Box','on');
        
        hold(ax2,'on');
        
        plot(ax2,time_HoT,omega_HoT(:,1),...
            'LineWidth',LW);
        
        plot(ax2,time_HoT,omega_HoT(:,2),...
            'LineWidth',LW);
        
        plot(ax2,time_HoT,omega_HoT(:,3),...
            'LineWidth',LW);
        
        grid(ax2,'on');
        
        xlim(ax2,xZoom);
        ylim(ax2,yZoom);
        
        set(ax2,...
            'FontSize',8,...
            'LineWidth',0.75,...
            'XTick',[0  1 2 3 4 ],...
            'YTick',[-11 -5  -1  1.9]);
        
        % Keep inset visible
        uistack(ax2,'top');
        
        %% Apply formatting
        set(findall(gcf,'-property','FontSize'),'FontSize',FS);
        
        %% ------------------------------------------
        %  Figure 10: Rotor speeds (HoT run)
        %  ------------------------------------------
        Omega_min = data_HoT.Omega_min;
        Omega_max = data_HoT.Omega_max;
        Omega_HoT = controls_HoT;
        
        figure;
        i=1;
        subplot(4,1,i);
        plot(time_HoT, sqrt(Omega_HoT(:,i)), 'LineWidth', LW/2,'HandleVisibility','off');
        yline(sqrt(Omega_min), '--k', 'LineWidth', LW);
        yline(sqrt(Omega_max), '--r', 'LineWidth', LW);
        ylim([sqrt(Omega_min) - 10, sqrt(Omega_max) + 10]);
        ylabel(['\Omega_', num2str(i),'HoT (Hz)']); grid on;
        lgd = legend('$\underline{\Omega}$','$\bar{\Omega}$','Interpreter','latex','Orientation','horizontal');
        lgd.FontSize = FSL; 
        

        for i = 2:4
            subplot(4,1,i);
            plot(time_HoT, sqrt(Omega_HoT(:,i)), 'LineWidth', LW/2,'HandleVisibility','off');
            yline(sqrt(Omega_min), '--k', 'LineWidth', LW,'HandleVisibility','off');
            yline(sqrt(Omega_max), '--r', 'LineWidth', LW,'HandleVisibility','off');
            ylim([sqrt(Omega_min) - 10, sqrt(Omega_max) + 10]);
            ylabel(['\Omega_', num2str(i),'HoT (Hz)']); grid on;
        end
        xlabel('Time (s)');

        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        

        %% ------------------------------------------
        %  Figure 11: Euler angles (HoT run)
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
        plot(time_HoT, roll,  'LineWidth', LW, 'DisplayName', 'Roll (HoT)');
        plot(time_HoT, pitch, 'LineWidth', LW, 'DisplayName', 'Pitch (HoT)');
        plot(time_HoT, yaw,   'LineWidth', LW, 'DisplayName', 'Yaw (HoT)');
        xlabel('Time (s)'); ylabel('Euler angles (deg)');
        legend('show'); grid on; 
        
        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        
        hold off;

        %% ------------------------------------------
        %  Figure 12: Position tracking error(HoT run)
        %  ------------------------------------------
        
        %% ============================================================
        %          BUILD REFERENCE STATE
        % ============================================================
        state_ref=zeros(size(time_HoT,2),13);
        state_ref(:,1:2) = pathXY_ProxyUtility(1:size(time_HoT,2),:);  % p_ref(1:2)
        state_ref(:,3)   = h_UAV*ones(size(time_HoT,2), 1);            % p_ref(3)
        state_ref(:,4)   = 1*ones(size(time_HoT,2), 1);                % q_ref = [1 0 0 0]
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
        tiledlayout(4, 1,'Padding','compact','TileSpacing','compact');
        
        % --- Position Tracking Error Norm ---
        nexttile;
        plot(time_HoT, err_p_HoT_norm, 'b', 'LineWidth', LW, 'DisplayName', 'HoT');
        hold on;
        plot(time_NoT, err_p_NoT_norm, 'r-.', 'LineWidth', LW, 'DisplayName', 'NoT');
        grid on; ylabel('$$\|p - p_d\|$$ (m)','Interpreter','latex');
        legend('show');
        
        % --- Velocity Tracking Error Norm ---
        nexttile;
        plot(time_HoT, err_v_HoT_norm, 'b', 'LineWidth', LW, 'DisplayName', 'HoT');
        hold on;
        plot(time_NoT, err_v_NoT_norm, 'r-.', 'LineWidth', LW, 'DisplayName', 'NoT');
        grid on; ylabel('$$\|v- v_d\|$$ (m/s)','Interpreter','latex');
        
        % --- Angular Velocity Tracking Error Norm ---
        nexttile;
        plot(time_HoT, err_w_HoT_norm, 'b', 'LineWidth', LW, 'DisplayName', 'HoT');
        hold on;
        plot(time_NoT, err_w_NoT_norm, 'r-.', 'LineWidth', LW, 'DisplayName', 'NoT');
        grid on; ylabel('$$\|\omega - \omega_d\|$$ (rad/s)','Interpreter','latex');
        
        % --- Orientation Geodesic Error ---
        nexttile;
        plot(time_HoT, err_q_HoT, 'b', 'LineWidth', LW, 'DisplayName', 'HoT');
        hold on;
        plot(time_NoT, err_q_NoT, 'r-.', 'LineWidth', LW, 'DisplayName', 'NoT');
        grid on; ylabel('$$\|\log(q \circ q_d^\star)\|$$','Interpreter','latex');
        xlabel('Time (s)');

        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);

        %% ============================================================
        %      OBJECTIVE DECOMPOSITION — HoT
        % ============================================================
        
        Q_vec = data_HoT.q_0;      % weight vector used in model
        Q_diag = diag(Q_vec);
        
        % Extract signals
        p     = p_HoT;
        v     = v_HoT;
        sv    = controls_HoT(:,5:4+K_User);
        omega = w_HoT;
        eta   = err_q_HoT;   % geodesic distance already computed
        
        % References
        p_ref = state_ref(:,1:3);
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
        tiledlayout(6, 1,'Padding','compact','TileSpacing','compact');
        
        % ------------------------------------------------------------
        nexttile;
        plot(time_HoT, J_p, 'b','LineWidth', LW);
        grid on;
        ylabel('J_p');
        title('Position Contribution');
        
        % ------------------------------------------------------------
        nexttile;
        plot(time_HoT, J_v, 'Color',[0.85 0.33 0.1],'LineWidth', LW);
        grid on;
        ylabel('J_v');
        title('Velocity Contribution');
        
        % ------------------------------------------------------------
        nexttile;
        plot(time_HoT, J_sv, 'm','LineWidth', LW);
        grid on;
        ylabel('J_{sv}');
        title('Slack (QoS) Contribution');
        
        % ------------------------------------------------------------
        nexttile;
        plot(time_HoT, J_omega, 'g','LineWidth', LW);
        grid on;
        ylabel('J_\omega');
        title('Angular Velocity Contribution');
        
        % ------------------------------------------------------------
        nexttile;
        plot(time_HoT, J_eta, 'k','LineWidth', LW);
        grid on;
        ylabel('J_\eta');
        title('Orientation (Geodesic) Contribution');
        
        % ------------------------------------------------------------
        nexttile;
        plot(time_HoT, J_total, 'LineWidth', LWth);
        grid on;
        ylabel('J_{total}');
        xlabel('Time (s)');
        title('Total Objective');
        
        % Link all x-axes
        ax = findall(gcf,'Type','axes');
        linkaxes(ax,'x');

        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);


        %% ============================================================
        % QoS VIOLATION STATISTICS
        %% ============================================================
        
        % ---------- Satisfaction probability ----------
        sat_matrix = (RATES_HoT >= R_min);
        
        QoS_satisfaction_probability = ...
            100 * sum(sat_matrix(:)) / numel(sat_matrix);
        
        % ---------- Average slack ----------
        slack_HoT = sv;
        
        average_slack = mean(slack_HoT(:));
        
        % ---------- Maximum slack ----------
        max_slack = max(slack_HoT(:));
        
        % ---------- Maximum violation depth ----------
        violation_depth = max(0, R_min - RATES_HoT);
        
        max_violation_depth = max(violation_depth(:));
        
        average_violation_depth = mean(violation_depth(:));
        
        fprintf('\nQoS Statistics\n');
        fprintf('Satisfaction probability = %.2f %%\n', ...
                QoS_satisfaction_probability);
        fprintf('Average slack = %.4f\n', average_slack);
        fprintf('Maximum slack = %.4f\n', max_slack);
        fprintf('Average violation depth = %.3f bit/s\n', ...
                average_violation_depth);
        fprintf('Maximum violation depth = %.3f bit/s\n', ...
                max_violation_depth);


        % Plot 1: Satisfaction indicator
        figure;
        
        imagesc(t_plot,1:K_User,sat_matrix');
        
        colormap([1 0.6 0.6;
                  0.6 1 0.6]);
        
        xlabel('Time (s)');
        ylabel('User index');
        
        title('QoS Satisfaction Map');
        
        cb = colorbar;
        cb.Ticks = [0.25 0.75];
        cb.TickLabels = {'Violation','Satisfied'};

        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);

        % Plot 2: Slack variables

        figure;
        hold on;
        
        for i = 1:K_User
            plot(time_HoT,slack_HoT(:,i), ...
                 'LineWidth', LW, ...
                 'DisplayName',['User ',num2str(i)]);
        end
        
        xlabel('Time (s)');
        ylabel('Slack variable');
        
        legend show;
        grid on;

        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);

        % %% ------------------------------------------
        % %  Plot: Average Slack Variable over Time
        % %  ------------------------------------------
        % 
        % % Extract slack variables from the control matrices
        % slack_HoT_all = controls_HoT(:, 5:4+K_User);
        % slack_NoT_all = controls_NoT(:, 5:4+K_User);
        % slack_SL_all = controls_SL(:, 5:4+K_User);
        % 
        % % Compute average across all users at each time step
        % mean_slack_HoT = mean(slack_HoT_all, 2);
        % mean_slack_NoT = mean(slack_NoT_all, 2);
        % mean_slack_SL = mean(slack_SL_all, 2);
        % 
        % figure; 
        % hold on; 
        % box on;
        % 
        % % =========================
        % % HoT Average Slack
        % % =========================
        % h_slack_hot = plot(time_HoT, mean_slack_HoT, ...
        %      'Color', blue, ...
        %      'LineWidth', LWth, ...
        %      'LineStyle', '-');
        % 
        % % =========================
        % % NoT Average Slack
        % % =========================
        % h_slack_not = plot(time_NoT, mean_slack_NoT, ...
        %      'Color', orange, ...
        %      'LineWidth', LW, ...
        %      'LineStyle', '--');
        % 
        % % =========================
        % % SL Average Slack
        % % =========================
        % h_slack_sl = plot(time_SL, mean_slack_SL, ...
        %      'Color', green, ...
        %      'LineWidth', LW, ...
        %      'LineStyle', '-.');
        % 
        % % % Re-plot HoT on top to ensure it is clearly visible
        % % plot(time_HoT, mean_slack_HoT, ...
        % %      'Color', blue, ...
        % %      'LineWidth', LW, ...
        % %      'LineStyle', '-');
        % % 
        % xlabel('Time (s)', 'FontSize', FS);
        % ylabel('Average Slack Variable', 'FontSize', FS);
        % 
        % grid on;
        % 
        % % Add Legend
        % legend([h_slack_hot, h_slack_not, h_slack_sl], ...
        %        {'HoT (Average Slack)', 'NoT (Average Slack)','SL (Average Slack)'}, ...
        %        'Location', 'northoutside', 'NumColumns', 2);
        % 
        % 
        % %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        % set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        % set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        % 
        % hold off;



        %% ------------------------------------------
        %  Plot: Normalized Average HoT Slack Variable
        %  ------------------------------------------
        
        % Extract HoT slack variables
        slack_HoT_all = controls_HoT(:, 5:4+K_User);
        
        % Compute average across all users at each time step
        mean_slack_HoT = mean(slack_HoT_all, 2);
        
        % Normalize by the maximum average HoT value
        mean_slack_HoT_norm = mean_slack_HoT / max(mean_slack_HoT);
        
        figure;
        hold on;
        box on;
        
        plot(time_HoT, mean_slack_HoT_norm, ...
            'Color', blue, ...
            'LineWidth', LWth, ...
            'LineStyle', '-');
        ylim([0 1.1]);
        
        xlabel('Time (s)', 'FontSize', FS);
        ylabel('Normalized Average HoT Slack', 'FontSize', FS);
        
        grid on;
        
        legend('HoT (Normalized)', ...
               'Location', 'best');
        
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        
        hold off;


        % Plot 3: Violation depth

        figure;
        hold on;
        
        for i = 1:K_User
            plot(t_plot, ...
                 violation_depth(:,i), ...
                 'LineWidth', LW, ...
                 'DisplayName',['User ',num2str(i)]);
        end
        
        xlabel('Time (s)');
        ylabel('Violation depth HoT (bit/s)');
        
        legend show;
        grid on;

        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        
        
        %% HoT: Histogram of NMPC solve times (Complexity Figure)
        figure;
        hHistHoT = histogram(solve_time_HoT, ...
                  'Normalization','percentage', ...
                  'NumBins',50);
        
        hold on;
        
        hTsHoT = xline(Ts, 'r', 'LineWidth', LWth);
        
        xlabel('Time (s)', 'Interpreter','latex', 'FontSize', FS);
        ylabel('\% control steps of HoT', 'Interpreter','latex', 'FontSize', FS);
        
        legend([hHistHoT, hTsHoT], ...
               {'Solve time distribution', '$T_s$ (sampling time)'}, ...
               'Interpreter','latex', 'Location','northeast', 'FontSize', FS);
        
        grid on;
        box on;
        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);
        
        %% NoT: Histogram of NMPC solve times (Complexity Figure)
        figure;
        hHistNoT = histogram(solve_time_NoT, ...
                  'Normalization','percentage', ...
                  'NumBins',50);
        
        hold on;
        
        hTsNoT = xline(Ts, 'r', 'LineWidth', LWth);
        
        xlabel('Time (s)', 'Interpreter','latex', 'FontSize', FS);
        ylabel('\% control steps of NoT', 'Interpreter','latex', 'FontSize', FS);
        
        legend([hHistNoT, hTsNoT], ...
               {'Solve time distribution', '$T_s$ (sampling time)'}, ...
               'Interpreter','latex', 'Location','northeast', 'FontSize', FS);
        
        grid on;
        box on;
        %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
        set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW);


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