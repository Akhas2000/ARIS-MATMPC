clear all; clear mex; close all; clc;

%% Parameters
K_set   = [2 4 6 8 10];     % Number of users to test
N_A     = 3;                % Antenna elements
M_size  = [4, 4];           % RIS size
N_iter  = 5;                % Number of random realizations per K
R_min=1*1e6;




% Storage for results
Tot_HoT_BP = zeros(length(K_set),1);
Tot_NoT_BP = zeros(length(K_set),1);
Tot_HoT_P  = zeros(length(K_set),1);
Tot_NoT_P  = zeros(length(K_set),1);
Tot_HoT_B  = zeros(length(K_set),1);
Tot_NoT_B  = zeros(length(K_set),1);

% --- New Benchmarks Storage ---
Tot_SL_BP  = zeros(length(K_set),1);
Tot_HOV_BP = zeros(length(K_set),1);

%% Main Loop
for idx = 1:length(K_set)

    K_User = K_set(idx);
    disp("----------------------------------------------------");
    disp(['Running for K = ', num2str(K_User)]);

    temp_HoT_BP = zeros(N_iter,1);
    temp_NoT_BP = zeros(N_iter,1);
    temp_HoT_P  = zeros(N_iter,1);
    temp_NoT_P  = zeros(N_iter,1);
    temp_HoT_B  = zeros(N_iter,1);
    temp_NoT_B  = zeros(N_iter,1);
    
    % --- Temp Arrays for New Benchmarks ---
    temp_SL_BP  = zeros(N_iter,1);
    temp_HOV_BP = zeros(N_iter,1);

    t = 1;
    while t <= N_iter
        try
            % ---- Generate network configuration ----
            Model_Generation_Scale(R_min,K_User, N_A, M_size);

            % ---- Run the main algorithm ----
            [Trans_HoT_BP, Trans_NoT_BP, ...
             Trans_HoT_P,  Trans_NoT_P, ...
             Trans_HoT_B,  Trans_NoT_B, ...
             Trans_SL_BP,  Trans_HOV_BP] = Main_Scale();

            % ---- Store results if success ----
            temp_HoT_BP(t) = Trans_HoT_BP;
            temp_NoT_BP(t) = Trans_NoT_BP;
            temp_HoT_P(t)  = Trans_HoT_P;
            temp_NoT_P(t)  = Trans_NoT_P;
            temp_HoT_B(t)  = Trans_HoT_B;
            temp_NoT_B(t)  = Trans_NoT_B;
            
            % --- Store New Benchmarks ---
            temp_SL_BP(t)  = Trans_SL_BP;
            temp_HOV_BP(t) = Trans_HOV_BP;

            disp(['→ Iteration t=' num2str(t) ' completed successfully.']);
            t = t + 1;   % Only increment after success

        catch ME
            disp(['ERROR at K=' num2str(K_User) ...
                  ', iteration t=' num2str(t)]);
            disp(ME.message);
            disp('Retrying the same iteration...');
        end
    end

    % ---- Average over the successful iterations ----
    Tot_HoT_BP(idx) = mean(temp_HoT_BP);
    Tot_NoT_BP(idx) = mean(temp_NoT_BP);
    Tot_HoT_P(idx)  = mean(temp_HoT_P);
    Tot_NoT_P(idx)  = mean(temp_NoT_P);
    Tot_HoT_B(idx)  = mean(temp_HoT_B);
    Tot_NoT_B(idx)  = mean(temp_NoT_B);
    
    % --- Average New Benchmarks ---
    Tot_SL_BP(idx)  = mean(temp_SL_BP);
    Tot_HOV_BP(idx) = mean(temp_HOV_BP);
end

%% ---- Plot Results ----


%% ============================
%  GLOBAL STYLE SETTINGS (for LaTeX-ready figures)
%  Figures are exported at their final print size (3.5 in wide,
%  single-column) so fonts/lines do NOT get shrunk when included
%  in LaTeX. Tune FIGSIZE if your column width differs.
%  ============================
FS      = 20;                % axis / tick label font size
FSL     = 16;                 % legend font size
LW      = 2.5;                % standard line width
LWth    = 3.5;                 % thicker/emphasis line width
MS      = 9;                  % standard marker size
MS_big  = 12;                  % larger marker size (scatter highlights)
AXLW    = 1.5;                 % axis box line width
FIGSIZE = [0 0 3.5 2.8];        % figure physical size in inches (match \includegraphics width)



% FS      = 10;               % axis / tick label font size (MATLAB default)
% FSL     = 9;                 % legend font size (MATLAB default, slightly below axis)
% LW      = 0.5;               % standard line width (MATLAB default)
% LWth    = 0.5;                % MATLAB has no separate "thick" default — same as LW
% MS      = 6;                 % standard marker size (MATLAB default)
% MS_big  = 6;                  % MATLAB has no separate "big" default — same as MS
% AXLW    = 0.5;                % axis box line width (MATLAB default)
% FIGSIZE = [0 0 8 6];           % MATLAB default figure size in inches (560x420 px screen size ≈ this)

% Define specific colors for the new plots to match the previous aesthetic
purple = [148 0 211]/255;
teal   = [0 128 128]/255;
figure; hold on; grid on; box on;
plot(K_set, Tot_HoT_BP, '-o', 'LineWidth', LW, 'DisplayName', 'HoT-BP');
plot(K_set, Tot_NoT_BP, '-s', 'LineWidth', LW, 'DisplayName', 'NoT-BP');
plot(K_set, Tot_HoT_P,  '-^', 'LineWidth', LW, 'DisplayName', 'HoT-P');
plot(K_set, Tot_NoT_P,  '-v', 'LineWidth', LW, 'DisplayName', 'NoT-P');
% Plot the new Benchmarks
plot(K_set, Tot_SL_BP,  '-*', 'Color', purple, 'LineWidth', LW, 'DisplayName', 'SL-BP');
%plot(K_set, Tot_HOV_BP, '-p', 'Color', teal,   'LineWidth', LW, 'DisplayName', 'HOV-BP');
plot(K_set, Tot_HoT_B,  '-d', 'LineWidth', LW, 'DisplayName', 'HoT-B');
plot(K_set, Tot_NoT_B,  '-x', 'LineWidth', LW, 'DisplayName', 'NoT-B');
xlabel('Number of Users (I)');
ylabel('Total Transmitted Data');
%title('Performance Comparison of All Schemes');
legend('Location', 'Best');
set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', AXLW);



save("Data_Scale_K_USER_Vs_Transmitted_Data.mat")
