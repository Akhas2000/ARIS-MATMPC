function [P_bar, Theta_bar, F_phases_bar, obj_evolution] = ...
    PSCA_REF_NoReg(settings, p_init, p_fin, T_total)

    %% Default Arguments
    if nargin < 4
        T_total = 45.0;
    end
    if nargin < 3
        p_fin = [-100; 100; 100];
    end
    if nargin < 2
        p_init = [-100; -100; 100];
    end

    %% Extract Settings

    pA        = settings.pA;
    pR        = settings.pR;
    pK        = settings.pK;
    Power     = settings.Power;
    Bandwidth = settings.Bandwidth;
    beta_R    = settings.beta_R;
    beta_B    = settings.beta_B;
    freq      = settings.freq;
    M_ele     = settings.M_ele;
    N_A       = settings.N_A;

    K_User = size(pK,2);

    %% Trajectory Setup

    z_fixed = p_init(3);

    v_max   = 17;     
    N_steps = 40;
    dt      = T_total/N_steps;

    R_rot = eye(3);

    %% fmincon Options

    options = optimoptions('fmincon', ...
        'Algorithm','interior-point', ...
        'MaxIterations',100, ...
        'Display','off');

    %% PSCA Parameters

    MAX_ITER = 15;
    gamma_step = 1.0;
    epsilon_diminish = 0.05;
    tol = 1e-2;

    %% Initialization

    P_bar = zeros(3,N_steps);

    for k = 1:N_steps
        alpha = (k-1)/(N_steps-1);
        P_bar(:,k) = (1-alpha)*p_init + alpha*p_fin;
    end

    Theta_bar = zeros(M_ele,N_steps);

    F_phases_bar = zeros(N_A,N_steps);

    for t = 1:N_steps
        F_phases_bar(:,t) = ...
            array_response_phases_BS( ...
            pA, P_bar(:,t), freq);
    end

    disp('======================================================');
    disp('     Starting 3-Block PSCA (NO REGULARIZATION)        ');
    disp('======================================================');

    obj_evolution = zeros(MAX_ITER,1);

    %% Main PSCA Loop

    for iter = 1:MAX_ITER

        fprintf('--- Iteration %d ---\n',iter);

        Theta_star    = zeros(M_ele,N_steps);
        F_phases_star = zeros(N_A,N_steps);

        % =====================================================
        % BLOCK 1 : RIS Optimization
        % =====================================================

        for t = 1:N_steps

            p_current = P_bar(:,t);
            f_current = F_phases_bar(:,t);

            theta_guess = Theta_bar(:,t);

            obj_theta = @(theta) ...
                -sum(Rates_No_Complex_phase( ...
                pA,pR,p_current,pK,R_rot,...
                theta,f_current,...
                Power,Bandwidth,freq,...
                beta_B,beta_R));

            lb_theta = zeros(M_ele,1);
            ub_theta = 2*pi*ones(M_ele,1);

            Theta_star(:,t) = fmincon( ...
                obj_theta,...
                theta_guess,...
                [],[],[],[],...
                lb_theta,ub_theta,...
                [],options);

        end

        % =====================================================
        % BLOCK 2 : Beamforming Optimization
        % =====================================================

        for t = 1:N_steps

            p_current = P_bar(:,t);

            theta_current = Theta_bar(:,t);

            f_guess = F_phases_bar(:,t);

            obj_f = @(f) ...
                -sum(Rates_No_Complex_phase( ...
                pA,pR,p_current,pK,R_rot,...
                theta_current,f,...
                Power,Bandwidth,freq,...
                beta_B,beta_R));

            lb_f = zeros(N_A,1);
            ub_f = 2*pi*ones(N_A,1);

            F_phases_star(:,t) = fmincon( ...
                obj_f,...
                f_guess,...
                [],[],[],[],...
                lb_f,ub_f,...
                [],options);

        end

        % =====================================================
        % BLOCK 3 : Position Optimization
        % =====================================================

        P_2D_bar = P_bar(1:2,:);
        P_guess_vec = P_2D_bar(:);

        Aeq = zeros(4,2*N_steps);

        Aeq(1:2,1:2) = eye(2);
        Aeq(3:4,end-1:end) = eye(2);

        beq = [p_init(1:2); p_fin(1:2)];

        obj_P = @(P_2D_vec) ...
            P_Objective_2D_NoReg( ...
            P_2D_vec,...
            z_fixed,...
            Theta_bar,...
            F_phases_bar,...
            pA,pR,pK,R_rot,...
            Power,Bandwidth,freq,...
            beta_B,beta_R);

        nonlcon = @(P_2D_vec) ...
            velocity_constraints_2D( ...
            P_2D_vec,...
            v_max*dt,...
            N_steps);

        P_star_vec = fmincon( ...
            obj_P,...
            P_guess_vec,...
            [],[],...
            Aeq,beq,...
            [],[],...
            nonlcon,...
            options);

        P_2D_star = reshape(P_star_vec,2,N_steps);

        P_star = [ ...
            P_2D_star;
            z_fixed*ones(1,N_steps)];

        % =====================================================
        % Update
        % =====================================================

        diff_P     = max(max(abs(P_star-P_bar)));
        diff_Theta = max(max(abs(Theta_star-Theta_bar)));
        diff_F     = max(max(abs(F_phases_star-F_phases_bar)));

        P_bar        = P_bar + gamma_step*(P_star-P_bar);
        Theta_bar    = Theta_bar + gamma_step*(Theta_star-Theta_bar);
        F_phases_bar = F_phases_bar + gamma_step*(F_phases_star-F_phases_bar);

        P_2D_eval = P_bar(1:2,:);

        total_network_rate = ...
            -P_Objective_2D_NoReg( ...
            P_2D_eval(:),...
            z_fixed,...
            Theta_bar,...
            F_phases_bar,...
            pA,pR,pK,R_rot,...
            Power,Bandwidth,freq,...
            beta_B,beta_R);

        obj_evolution(iter) = total_network_rate/1e6;

        fprintf('  Step Size     = %.3f\n',gamma_step);
        fprintf('  Max P diff    = %.4f m\n',diff_P);
        fprintf('  Max Th diff   = %.4f rad\n',diff_Theta);
        fprintf('  Max F diff    = %.4f rad\n',diff_F);
        fprintf('  Total SumRate = %.2f Mbps\n',obj_evolution(iter));

        if diff_P < tol && diff_Theta < tol && diff_F < tol

            disp('Converged.');

            obj_evolution = obj_evolution(1:iter);

            break;

        end

        gamma_step = gamma_step * ...
            (1 - epsilon_diminish*gamma_step);

    end

    disp('======================================================');
    disp('                Optimization Complete                 ');
    disp('======================================================');

end

%% ============================================================
%% Position Objective (NO REGULARIZATION)
%% ============================================================

function cost = P_Objective_2D_NoReg( ...
    P_2D_vec,...
    z_fixed,...
    Theta_bar,...
    F_phases_bar,...
    pA,pR,pK,R_rot,...
    Power,Bandwidth,freq,...
    beta_B,beta_R)

    N = numel(P_2D_vec)/2;

    P_2D = reshape(P_2D_vec,2,N);

    P = [P_2D; z_fixed*ones(1,N)];

    total_rate = 0;

    for t = 1:N

        Rates = Rates_No_Complex_phase( ...
            pA,pR,P(:,t),pK,R_rot,...
            Theta_bar(:,t),...
            F_phases_bar(:,t),...
            Power,Bandwidth,freq,...
            beta_B,beta_R);

        total_rate = total_rate + sum(Rates);

    end

    cost = -total_rate;

end

%% ============================================================
%% Velocity Constraints
%% ============================================================

function [c,ceq] = velocity_constraints_2D( ...
    P_2D_vec,...
    max_dist,...
    N)

    P_2D = reshape(P_2D_vec,2,N);

    c = zeros(N-1,1);

    for t = 1:N-1

        c(t) = ...
            norm(P_2D(:,t+1)-P_2D(:,t)) ...
            - max_dist;

    end

    ceq = [];

end