function [pU_opt, theta_opt, f_opt, obj_evolution] = ...
    PSCA_Optimal_Position(settings, pU_init)

% ==========================================================
% Parameters
% ==========================================================
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

R_rot = eye(3);

options = optimoptions('fmincon', ...
    'Algorithm','interior-point', ...
    'Display','off');

MAX_ITER = 20;
tol      = 1e-3;

% ==========================================================
% Initialization
% ==========================================================
pU_bar = pU_init(:);          % Ensure column vector

theta_bar = zeros(M_ele,1);   % RIS phases

f_bar = zeros(N_A,1);         % Beamforming variables

obj_evolution = zeros(MAX_ITER,1);

% ==========================================================
% Main Loop
% ==========================================================
for iter = 1:MAX_ITER

    %% =====================================================
    % Block 1 : RIS Optimization
    %% =====================================================

    obj_theta = @(theta) ...
        -sum(Rates_No_Complex_phase( ...
        pA, pR, pU_bar, pK, R_rot, ...
        theta, f_bar, ...
        Power, Bandwidth, freq, ...
        beta_B, beta_R));

    theta_star = fmincon( ...
        obj_theta, ...
        theta_bar, ...
        [], [], [], [], ...
        zeros(M_ele,1), ...
        2*pi*ones(M_ele,1), ...
        [], options);

    %% =====================================================
    % Block 2 : Beamforming Optimization
    %% =====================================================

    obj_f = @(f) ...
        -sum(Rates_No_Complex_phase( ...
        pA, pR, pU_bar, pK, R_rot, ...
        theta_star, f, ...
        Power, Bandwidth, freq, ...
        beta_B, beta_R));

    f_star = fmincon( ...
        obj_f, ...
        f_bar, ...
        [], [], [], [], ...
        zeros(N_A,1), ...
        2*pi*ones(N_A,1), ...
        [], options);

    %% =====================================================
    % Block 3 : UAV Position Optimization
    %% =====================================================

    obj_p = @(p) ...
        -sum(Rates_No_Complex_phase( ...
        pA, pR, p(:), pK, R_rot, ...
        theta_star, f_star, ...
        Power, Bandwidth, freq, ...
        beta_B, beta_R));

    % UAV position bounds
    lb = [-200; -200; 100];
    ub = [ 200;  200; 100];   % Fixed altitude = 100 m

    p_star = fmincon( ...
        obj_p, ...
        pU_bar, ...
        [], [], [], [], ...
        lb, ub, ...
        [], options);

    %% =====================================================
    % Convergence Check
    %% =====================================================

    diff_theta = norm(theta_star - theta_bar);
    diff_f     = norm(f_star - f_bar);
    diff_p     = norm(p_star - pU_bar);

    theta_bar = theta_star;
    f_bar     = f_star;
    pU_bar    = p_star;

    rate = sum( ...
        Rates_No_Complex_phase( ...
        pA, pR, pU_bar, pK, R_rot, ...
        theta_bar, f_bar, ...
        Power, Bandwidth, freq, ...
        beta_B, beta_R));

    obj_evolution(iter) = rate/1e6;

    fprintf('Iter %2d | Sum Rate = %.4f Mbps\n', ...
        iter, obj_evolution(iter));

    if max([diff_theta, diff_f, diff_p]) < tol
        obj_evolution = obj_evolution(1:iter);
        break;
    end

end

% Trim history if maximum iterations reached
obj_evolution = obj_evolution(1:iter);

% ==========================================================
% Outputs
% ==========================================================
theta_opt = theta_bar;
f_opt     = f_bar;
pU_opt    = pU_bar;

end