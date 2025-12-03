clear all; clear mex; close all; clc;

%% Parameters
K_set   = [2 4 6 8 10];     % Number of users to test
N_A     = 3;                % Antenna elements
M_size  = [4, 4];           % RIS size
N_iter  = 5;                % Number of random realizations per K

% Storage for results
Tot_HoT_BP = zeros(length(K_set),1);
Tot_NoT_BP = zeros(length(K_set),1);
Tot_HoT_P  = zeros(length(K_set),1);
Tot_NoT_P  = zeros(length(K_set),1);
Tot_HoT_B  = zeros(length(K_set),1);
Tot_NoT_B  = zeros(length(K_set),1);

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

    t = 1;
    while t <= N_iter
        try
            % ---- Generate network configuration ----
            Model_Generation_Scale(K_User, N_A, M_size);

            % ---- Run the main algorithm ----
            [Trans_HoT_BP,Trans_NoT_BP, ...
             Trans_HoT_P, Trans_NoT_P, ...
             Trans_HoT_B, Trans_NoT_B] = Main_Scale();

            % ---- Store results if success ----
            temp_HoT_BP(t) = Trans_HoT_BP;
            temp_NoT_BP(t) = Trans_NoT_BP;
            temp_HoT_P(t)  = Trans_HoT_P;
            temp_NoT_P(t)  = Trans_NoT_P;
            temp_HoT_B(t)  = Trans_HoT_B;
            temp_NoT_B(t)  = Trans_NoT_B;

            disp(['→ Iteration t=' num2str(t) ' completed successfully.']);
            t = t + 1;   % Only increment after success

        catch ME
            disp(['ERROR at K=' num2str(K_User) ...
                  ', iteration t=' num2str(t)]);
            disp(ME.message);
            disp('Retrying the same iteration...');
        end
    end

    % ---- Average over the 5 successful iterations ----
    Tot_HoT_BP(idx) = mean(temp_HoT_BP);
    Tot_NoT_BP(idx) = mean(temp_NoT_BP);
    Tot_HoT_P(idx)  = mean(temp_HoT_P);
    Tot_NoT_P(idx)  = mean(temp_NoT_P);
    Tot_HoT_B(idx)  = mean(temp_HoT_B);
    Tot_NoT_B(idx)  = mean(temp_NoT_B);
end

%% ---- Plot Results ----
figure; hold on; grid on; box on;



plot(K_set, Tot_HoT_BP,'-o','LineWidth',1.5,'DisplayName','HoT-BP');
plot(K_set, Tot_HoT_P, '-^','LineWidth',1.5,'DisplayName','HoT-P');
plot(K_set, Tot_NoT_BP,'-s','LineWidth',1.5,'DisplayName','NoT-BP');
plot(K_set, Tot_NoT_P, '-v','LineWidth',1.5,'DisplayName','NoT-P');
plot(K_set, Tot_HoT_B, '-d','LineWidth',1.5,'DisplayName','HoT-B');
plot(K_set, Tot_NoT_B, '-x','LineWidth',1.5,'DisplayName','NoT-B');

xlabel('Number of Users (K)');
ylabel('Total Transmitted Data');
%title('Performance Comparison of All Schemes');
legend('Location','Best');

save("Data_Scale_K_USER_Vs_Transmitted_Data.mat")
