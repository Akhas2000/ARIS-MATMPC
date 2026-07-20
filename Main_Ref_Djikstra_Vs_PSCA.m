clear all; clear mex; close all;clc;

% Add the path to the functions
addpath(genpath('Functions'));

disp( ' ' );
disp( 'MATMPC -- A (MAT)LAB based Model(M) Predictive(P) Control(C) Package.' );
disp( 'Copyright (C) 2016-2019 by Yutao Chen, University of Padova' );
disp( 'All rights reserved.' );
disp( ' ' );
disp( 'MATMPC is distributed under the terms of the' );
disp( 'GNU General Public License 3.0 in the hope that it will be' );
disp( 'useful, but WITHOUT ANY WARRANTY; without even the implied warranty' );
disp( 'of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.' );
disp( 'See the GNU General Public License for more details.' );
disp( ' ' );
disp( ' ' );
disp('---------------------------------------------------------------------------------');

%% Configuration (complete your configuration here...)
addpath([pwd,'/nmpc']); 
addpath([pwd,'/model_src']);
addpath([pwd,'/mex_core']);
addpath(genpath([pwd,'/data']));
if ismac
    addpath(genpath([pwd,'/solver/mac']));
elseif isunix
    addpath(genpath([pwd,'/solver/linux']));
elseif ispc
    addpath(genpath([pwd,'/solver/win64']));
else
    disp('Platform not supported')
end

cd data;
if exist('settings','file')==2
    load settings
    cd ..
else 
    cd ..
    error('No setting data is detected!');
end

Ts = settings.Ts_st;     % Closed-loop sampling time (usually = shooting interval)

Ts_st = settings.Ts_st;  % Shooting interval
nx = settings.nx;    % No. of states
nu = settings.nu;    % No. of controls
ny = settings.ny;    % No. of outputs (references)    
nyN= settings.nyN;   % No. of outputs at terminal stage 
np = settings.np;    % No. of parameters (on-line data)
nc = settings.nc;    % No. of constraints
ncN = settings.ncN;  % No. of constraints at terminal stage
nbx = settings.nbx;  % No. of state bounds



%% solver configurations

N  = 15;             % No. of shooting points
settings.N = N;

N2 = N/5;
settings.N2 = N2;    % No. of horizon length after partial condensing (N2=1 means full condensing)

r = 10;
settings.r = r;      % No. of input blocks (go to InitMemory.m, line 441 to configure)

opt.hessian         = 'Gauss_Newton';  % 'Gauss_Newton', 'Generalized_Gauss_Newton'
opt.integrator      = 'ERK4'; % 'ERK4','IRK3','IRK3-DAE'
opt.condensing      = 'default_full';  %'default_full','no','blasfeo_full(require blasfeo installed)','partial_condensing'
opt.qpsolver        = 'qpoases'; 
opt.hotstart        = 'no'; %'yes','no' (only for qpoases, use 'no' for nonlinear systems)
opt.shifting        = 'no'; % 'yes','no'
opt.ref_type        = 2; % 0-time invariant, 1-time varying(no preview), 2-time varying (preview)
opt.nonuniform_grid = 0; % if use non-uniform grid discretization (go to InitMemory.m, line 459 to configure)
opt.RTI             = 'yes'; % if use Real-time Iteration




%% available qpsolver

%'qpoases' (condensing is needed)
%'qpoases_mb' (move blocking strategy)
%'quadprog_dense' (for full condensing)
%'hpipm_sparse' (run mex_core/compile_hpipm.m first; set opt.condensing='no')
%'hpipm_pcond' (run mex_core/compile_hpipm.m first; set opt.condensing='no')
%'ipopt_dense' (install OPTI Toolbox first; for full condensing)
%'ipopt_sparse' (install OPTI Toolbox first; set opt.condensing='no')
%'ipopt_partial_sparse'(set opt.condensing='partial_condensing'; only for state and control bounded problems)
%'osqp_sparse' (set opt.condensing='no')
%'osqp_partial_sparse' (set opt.condensing='partial_condensing')
%'qpalm_cond' (condensing is needed)
%'qpalm_sparse'(set opt.condensing='no')


%% Reference Trajectory Generation: Only Added part to the original Simulation file of MATMPC

% Simulation Duration
Tf_init =80;  % simulation time

% ---------- AREA & RANDOM CLOUD SIZE ----------------------------------
xmin = -200;  xmax =  200;            % [m] rectangle in x
ymin = -200;  ymax =  200;            % [m] rectangle in y
h_UAV = 100;                          % [m] fixed altitude
Nrand = 30000;                        % << huge number of random nodes

R = eye(3);                           % rotation (identity)
M_ele=settings.M_ele;
N_A=settings.N_A;
pA=settings.pA; 
pR=settings.pR; 
pK=settings.pK; 
Power=settings.Power; 
Bandwidth=settings.Bandwidth; 
beta_R=settings.beta_R; 
beta_B=settings.beta_B; 
freq=settings.freq; 
p_bar_User=settings.p_bar_User; 
% ---------- RANDOM NODE CO-ORDINATES ----------------------------------

x_vals = xmin + (xmax-xmin)*rand(1,Nrand);   % 1×Nrand
y_vals = ymin + (ymax-ymin)*rand(1,Nrand);   % 1×Nrand
R_mat   = zeros(1,Nrand);                     % Σ-rate per node
Proxy_Utility_mat   = zeros(1,Nrand);                     % Σ-rate per node

% ---------- OPTIMAL TRAJECTORY (k-NN graph) ---------------------------
p_init  =  [-100 -100];         % start (physical coordinate)
p_final = [ -100  100];      % destination

x_vals(1)=p_init(1);x_vals(Nrand)=p_final(1);
y_vals(1)=p_init(2);y_vals(Nrand)=p_final(2);
% ---------- RATE CALCULATION ------------------------------------------
for n = 1:Nrand
    pU = [x_vals(n), y_vals(n), h_UAV];

    f_phases  = array_response_phases_BS(pA,pU,freq);  

    P_A_rx_RIS = phase_array_response_RIS(pR,pU,[],R,freq);
    P_A_tx_RIS = phase_array_response_RIS(pR,pU,p_bar_User,R,freq);
    theta      = P_A_tx_RIS - P_A_rx_RIS;

    Rates      = Rates_No_Complex_phase(pA,pR,pU,pK,R,theta,...
                                  f_phases,Power,Bandwidth,freq,...
                                  beta_B,beta_R);
    Rmat(n)    = sum(Rates);

    % Proxy Utility
    norm_diff = norm(pU - p_bar_User);
    norm_self = norm(pU);
    Proxy_Utility_mat(n) = (norm_diff*norm_self)^8;
end




% ---------- OPTIMAL TRAJECTORY (k-NN graph) ---------------------------

[pathXY] = optimalTrajectoryPU( ...
    Proxy_Utility_mat, x_vals, y_vals, ...
    p_init, p_final, ...
    'k', 8);

% ---- STEP 1: Smooth geometrically (no time yet) ----
Ns = floor(Tf_init / Ts_st);


% ---- STEP 2: Zero-order hold in time ----

Nw = size(pathXY,1);

samples_per_wp = floor(Ns / Nw);

pathZOH = [];

for i = 1:Nw
    block = repmat(pathXY(i,:), samples_per_wp, 1);
    pathZOH = [pathZOH; block];
end

% pad if necessary
while size(pathZOH,1) < Ns
    pathZOH = [pathZOH; pathXY(end,:)];
end

pathXY_ProxyUtility = pathZOH;
% ---- extend reference for preview safety ----
last_point = pathXY_ProxyUtility(end,:);

for i = 1:N
    pathXY_ProxyUtility = [pathXY_ProxyUtility; last_point];
end



Ns = size(pathXY_ProxyUtility,1);



% -------- HoT-PSCA : Horizontal orientation Tracking--------


p_init_PSCA  =  [p_init h_UAV];         % start (physical coordinate)
p_final_PSCA = [ p_final h_UAV];      % destination

N_horizon=N;


% Run the PSCA Function
[P_bar, Theta_bar, F_phases_bar, obj_evolution] = ...
    PSCA_REF_NoReg(settings, p_init_PSCA, p_final_PSCA, Tf_init);


smooth_win = 5;   % odd window length; increase for more smoothing

[Ref_Traj_ZOH, Ref_Theta_ZOH, Ref_Beam_ZOH, t_NMPC_ext] = ...
    PSCA_Interpolate(P_bar, Theta_bar, F_phases_bar, p_init_PSCA, p_final_PSCA, ...
                      Tf_init, Ts_st, N_horizon, smooth_win);

    



%% ============================
%  GLOBAL STYLE SETTINGS (for LaTeX-ready figures)
%  Figures are exported at their final print size (3.5 in wide,
%  single-column) so fonts/lines do NOT get shrunk when included
%  in LaTeX. Tune FIGSIZE if your column width differs.
%  ============================
% FS      = 20;                % axis / tick label font size
% FSL     = 16;                 % legend font size
% LW      = 2.5;                % standard line width
% LWth    = 3.5;                 % thicker/emphasis line width
% MS      = 9;                  % standard marker size
% MS_big  = 12;                  % larger marker size (scatter highlights)
% AXLW    = 1.5;                 % axis box line width
% FIGSIZE = [0 0 3.5 2.8];        % figure physical size in inches (match \includegraphics width)
% 

FS      = 10;                % axis / tick label font size
FSL     = 3;                 % legend font size
LW      = 1.5;                % standard line width
LWth    = 1.5;                 % thicker/emphasis line width
MS      = 6;                  % standard marker size
MS_big  = 6;                  % larger marker size (scatter highlights)
AXLW    = 0.5;                 % axis box line width
% FIGSIZE = [0 0 3.5 2.8];        % figure physical size in inches (match \includegraphics width)




% FS      = 10;               % axis / tick label font size (MATLAB default)
% FSL     = 9;                 % legend font size (MATLAB default, slightly below axis)
% LW      = 0.5;               % standard line width (MATLAB default)
% LWth    = 0.5;                % MATLAB has no separate "thick" default — same as LW
% MS      = 6;                 % standard marker size (MATLAB default)
% MS_big  = 6;                  % MATLAB has no separate "big" default — same as MS
% AXLW    = 0.5;                % axis box line width (MATLAB default)
% FIGSIZE = [0 0 8 6];           % MATLAB default figure size in inches (560x420 px screen size ≈ this)


%% ------------------------------------------
% Reference Trajectories Comparison
% ------------------------------------------

figure;

scatter(x_vals,y_vals,12,Rmat,'filled');
axis equal;
grid on;
colorbar;
xlabel('x (m)');
ylabel('y (m)');
hold on;

% BS
plot(0, 0,'rs','MarkerSize', MS_big,'MarkerFaceColor','r');

% Users
plot(pK(1,:),pK(2,:),'ro',...
    'MarkerSize', MS,...
    'MarkerFaceColor','r');

% Barycenter
plot(p_bar_User(1),p_bar_User(2),...
    'kp',...
    'MarkerSize', MS_big,...
    'MarkerFaceColor','m');

% Dijkstra reference
plot(pathXY_ProxyUtility(:,1),...
     pathXY_ProxyUtility(:,2),...
     'r-','LineWidth', LWth);

% PSCA reference
plot(Ref_Traj_ZOH(1,:),...
     Ref_Traj_ZOH(2,:),...
     'b--','LineWidth', LWth);

% Start
plot(pathXY_ProxyUtility(1,1),...
     pathXY_ProxyUtility(1,2),...
     'rs',...
     'MarkerFaceColor','r',...
     'HandleVisibility','off');

% Finish
plot(pathXY_ProxyUtility(end,1),...
     pathXY_ProxyUtility(end,2),...
     'rd',...
     'MarkerFaceColor','r',...
     'HandleVisibility','off');

legend({'Rate samples',...
        'BS',...
        'Users',...
        'User barycenter',...
        'PURG',...
        'PSCA-RG'},...
        'Location','northoutside',...
        'NumColumns',3);

cb = colorbar;
cb.Label.String = 'Total network rate (bit/s)';
cb.Label.FontSize = FSL;

%set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', AXLW);

hold off;

%% ------------------------------------------
%  Communication metrics along all flights
%  ------------------------------------------
            

% Initial BF
pU0 = p_init_PSCA;
f_phases_init = array_response_phases_BS(pA, pU0, freq);

R_fixed = eye(3);
theta_init = [];
P_A_rx_RIS0 = phase_array_response_RIS(pR, pU0, [],          R_fixed, freq);
P_A_tx_RIS0 = phase_array_response_RIS(pR, pU0, p_bar_User,  R_fixed, freq);
for m_idx = 1:M_ele
    theta_init = [theta_init; P_A_tx_RIS0(m_idx) - P_A_rx_RIS0(m_idx)];
end

% Storage





Total_DK_BP = []; Total_PSCA_BP = [];
RATES_DK_BP = []; RATES_PSCA_BP = [];
        
time_draw = 0;
iter_draw = 1;
imax=Ns;
Nsim = imax;

Tf=Tf_init;


while time_draw(end) < Tf && iter_draw <= imax


    % ---------- DK-BP ----------
    q0 = 1; q1 = 0;
    q2 = 0; q3 = 0;
    R_DK_BP = [ 1-2*(q2^2+q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
              2*(q1*q2 + q0*q3), 1-2*(q1^2+q3^2),     2*(q2*q3 - q0*q1);
              2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1),   1-2*(q1^2+q2^2) ];

    pU_DK_BP = [pathXY_ProxyUtility(iter_draw,1), pathXY_ProxyUtility(iter_draw,2), h_UAV];
    f_DK_BP  = array_response_phases_BS(pA, pU_DK_BP, freq);
    theta_DK_BP = phase_array_response_RIS(pR, pU_DK_BP, p_bar_User, R_DK_BP, freq) - ...
                phase_array_response_RIS(pR, pU_DK_BP, [], R_DK_BP, freq);

    % ---------- PSCA-BP ----------
    q0 = 1; q1 = 0;
    q2 = 0; q3 = 0;
    R_PSCA_BP = [ 1-2*(q2^2+q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
              2*(q1*q2 + q0*q3), 1-2*(q1^2+q3^2),     2*(q2*q3 - q0*q1);
              2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1),   1-2*(q1^2+q2^2) ];
    pU_PSCA_BP = [Ref_Traj_ZOH(1,iter_draw), Ref_Traj_ZOH(2,iter_draw), h_UAV];
    
    % PSCA tracks optimized RIS phases and BS Beamforming directly!
    theta_PSCA_BP = Ref_Theta_ZOH(:, iter_draw);
    f_PSCA_BP     = Ref_Beam_ZOH(:, iter_draw);

    % ---------- Rates ----------
   
    Rates_DK_BP_i = Rates_No_Complex_phase(pA,pR,pU_DK_BP,pK,R_DK_BP,theta_DK_BP,f_DK_BP, Power,Bandwidth,freq,beta_B,beta_R);
    Rates_PSCA_BP_i = Rates_No_Complex_phase(pA,pR,pU_PSCA_BP,pK,R_PSCA_BP,theta_PSCA_BP,f_PSCA_BP, Power,Bandwidth,freq,beta_B,beta_R);
    
    

    RATES_DK_BP = [RATES_DK_BP; Rates_DK_BP_i]; RATES_PSCA_BP = [RATES_PSCA_BP; Rates_PSCA_BP_i];

    Total_DK_BP = [Total_DK_BP; sum(   Rates_DK_BP_i)];
    Total_PSCA_BP = [Total_PSCA_BP; sum(Rates_PSCA_BP_i )];




    % time advance
    nextTime = iter_draw * Ts;
    iter_draw = iter_draw + 1;
    time_draw = [time_draw nextTime];
end


%% ------------------------------------------
%  Figure: Cumulative network rate for the reference trajectory,
%  sum_points sum_k R_points(k) (point is a point of the reference
%  trajectory)
%  ------------------------------------------
Trans_PSCA_BP = cumsum(Total_PSCA_BP) ;
Trans_DK_BP = cumsum(Total_DK_BP) ;
blue   = [0 0 255]/255;
red    = [220 20 60]/255;

figure; hold on;
plot(time_draw((1:end-1)),Trans_DK_BP, '-','Color',blue, 'LineWidth', LW, 'DisplayName','PURG');
plot(time_draw(1:end-1),Trans_PSCA_BP, '-','Color',red, 'LineWidth', LWth, 'DisplayName','PSCA-RG');



xlabel('Time (s)'); ylabel('Cumulative sumrate (bit/s)');
legend('show','Location','best'); grid on;


%set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', AXLW);




%% ------------------------------------------
% User data rates along the path
% ------------------------------------------

K = size(RATES_DK_BP,2);     % number of users
s = time_draw(1:end-1);   % path index

figure; hold on; grid on;

for k = 1:K
    % DK trajectory
    plot(s, RATES_DK_BP(:,k), '-', ...
        'LineWidth', LW, ...
        'DisplayName', sprintf('User %d (PURG)',k));

    % PSCA trajectory
    plot(s, RATES_PSCA_BP(:,k), '--', ...
        'LineWidth', LW, ...
        'DisplayName', sprintf('User %d (PSCA-RG)',k));
end

xlabel('Path point index');
ylabel('User rate (bit/s)');
%title('User data rates along the trajectory');

legend('Location','eastoutside');


%set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', AXLW);


% Plot Trajectory
figure('Name', 'PSCA Reference Trajectory'); hold on; grid on;
plot3(P_bar(1,:), P_bar(2,:), P_bar(3,:), 'b-o', 'LineWidth', LWth, 'MarkerSize', MS);
plot3(p_init_PSCA(1), p_init_PSCA(2), p_init_PSCA(3), 'gs', 'MarkerSize', MS_big, 'MarkerFaceColor', 'g');
plot3(p_final_PSCA(1), p_final_PSCA(2), p_final_PSCA(3), 'rs', 'MarkerSize', MS_big, 'MarkerFaceColor', 'r');
scatter3(pK(1,:), pK(2,:), pK(3,:), 112,'k', 'filled', '^');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
%title('3-Block PSCA Optimized Reference Trajectory');
legend('UAV Path', 'Start', 'End', 'Users');
view(3);


%set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', AXLW);


% Plot Objective Evolution
figure('Name', 'Objective Function Evolution');
plot(1:length(obj_evolution), obj_evolution, '-o', 'LineWidth', LWth, 'MarkerSize', MS);
grid on;
xlabel('PSCA Iteration');
ylabel('Total Network Sum Rate [Mbps]');
%title('Objective Function Evolution');
xlim([1, max(2, length(obj_evolution))]);


%set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', AXLW);


save("Main_Ref_Djikstra_Vs_PSCA.mat")
