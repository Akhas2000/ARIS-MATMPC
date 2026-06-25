clear all; clear mex; close all; clc;

%% Parameters
K_User = 5;                 % Fixed number of users
N_A    = 3;                 % Antenna elements
N_iter = 5;                 % Number of random realizations
R_min=1e4;

M_rows = 1:5;               % Sweep M = 1 to 5
M_cols = 4;                 % Always 4 columns



% Storage for results
Tot_HoT_BP = zeros(length(M_rows),1);
Tot_NoT_BP = zeros(length(M_rows),1);
Tot_HoT_P  = zeros(length(M_rows),1);
Tot_NoT_P  = zeros(length(M_rows),1);
Tot_HoT_B  = zeros(length(M_rows),1);
Tot_NoT_B  = zeros(length(M_rows),1);

% --- New Benchmarks Storage ---
Tot_SL_BP  = zeros(length(M_rows),1);
Tot_HOV_BP = zeros(length(M_rows),1);

%% Main Loop
for idx = 1:length(M_rows)

    M_size = [M_rows(idx), M_cols];
    disp("----------------------------------------------------");
    disp(['Running for M = ', num2str(M_rows(idx)), ...
          ', M_size = [', num2str(M_rows(idx)), ',', num2str(M_cols), ']']);

    % Temporary results for averaging
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

            % Store results if success
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
            t = t + 1;   % Move to next iteration

        catch ME
            % Error caught → retry same iteration
            disp(['ERROR at M=' num2str(M_rows(idx)) ...
                  ', iteration t=' num2str(t)]);
            disp(ME.message);
            disp('Retrying the same iteration...');
        end
    end

    % ---- Average over 5 successful runs ----
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

% Define specific colors for the new plots to match the previous aesthetic
purple = [148 0 211]/255;  
teal   = [0 128 128]/255;  

figure; hold on; grid on; box on;

% Create x-axis vector representing total number of RIS elements (M_rows * M_cols)
x_axis = M_rows * M_cols;

plot(x_axis, Tot_HoT_BP, '-o', 'LineWidth', 1.5, 'DisplayName', 'HoT-BP');
plot(x_axis, Tot_NoT_BP, '-s', 'LineWidth', 1.5, 'DisplayName', 'NoT-BP');

% Plot the new Benchmarks
plot(x_axis, Tot_SL_BP,  '-*', 'Color', purple, 'LineWidth', 1.5, 'DisplayName', 'SL-BP');
plot(x_axis, Tot_HOV_BP, '-p', 'Color', teal,   'LineWidth', 1.5, 'DisplayName', 'HOV-BP');

plot(x_axis, Tot_HoT_P,  '-^', 'LineWidth', 1.5, 'DisplayName', 'HoT-P');
plot(x_axis, Tot_NoT_P,  '-v', 'LineWidth', 1.5, 'DisplayName', 'NoT-P');
plot(x_axis, Tot_HoT_B,  '-d', 'LineWidth', 1.5, 'DisplayName', 'HoT-B');
plot(x_axis, Tot_NoT_B,  '-x', 'LineWidth', 1.5, 'DisplayName', 'NoT-B');


xlabel('Number of RIS elements (M)');
ylabel('Total Transmitted Data');
%title('Performance vs. RIS Size');
legend('Location','Best');


save("Data_Scale_M_size_Vs_Transmitted_Data.mat")