clear all; clear mex; close all; clc;

%% Parameters
K_User = 2;                 % Fixed number of users
N_A    = 3;                 % Antenna elements
N_iter = 5;                 % Number of random realizations

M_rows = 1:5;               % Sweep M = 1 to 5
M_cols = 4;                 % Always 4 columns

% Storage for results
Tot_HoT_BP = zeros(length(M_rows),1);
Tot_NoT_BP = zeros(length(M_rows),1);
Tot_HoT_P  = zeros(length(M_rows),1);
Tot_NoT_P  = zeros(length(M_rows),1);
Tot_HoT_B  = zeros(length(M_rows),1);
Tot_NoT_B  = zeros(length(M_rows),1);

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

    t = 1;
    while t <= N_iter
        try
            % ---- Generate network configuration ----
            Model_Generation_Scale(K_User, N_A, M_size);

            % ---- Run the main algorithm ----
            [Trans_HoT_BP,Trans_NoT_BP, ...
             Trans_HoT_P,Trans_NoT_P, ...
             Trans_HoT_B,Trans_NoT_B] = Main_Scale();

            % Store results if success
            temp_HoT_BP(t) = Trans_HoT_BP;
            temp_NoT_BP(t) = Trans_NoT_BP;
            temp_HoT_P(t)  = Trans_HoT_P;
            temp_NoT_P(t)  = Trans_NoT_P;
            temp_HoT_B(t)  = Trans_HoT_B;
            temp_NoT_B(t)  = Trans_NoT_B;

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
end


%% ---- Plot Results ----
figure; hold on; grid on; box on;

plot(M_rows*M_size(2), Tot_HoT_BP,'-o','LineWidth',1.5);
plot(M_rows*M_size(2), Tot_HoT_P, '-^','LineWidth',1.5);
plot(M_rows*M_size(2), Tot_NoT_BP,'-s','LineWidth',1.5);
plot(M_rows*M_size(2), Tot_NoT_P, '-v','LineWidth',1.5);
plot(M_rows*M_size(2), Tot_HoT_B, '-d','LineWidth',1.5);
plot(M_rows*M_size(2), Tot_NoT_B, '-x','LineWidth',1.5);

xlabel('Number of RIS elements (M)');
ylabel('Total Transmitted Data');
title('Performance vs. RIS Size (K = 2)');
legend('HoT-BP','NoT-BP','HoT-P','NoT-P','HoT-B','NoT-B', ...
       'Location','Best');


save("Data_Scale_M_size_Vs_Transmitted_Data.mat")
