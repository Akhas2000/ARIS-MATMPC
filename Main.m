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

N  = 50;             % No. of shooting points
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
Tf_init =60;  % simulation time

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
Proxy_Utility_mat   = zeros(1,Nrand);         % Σ-rate per node

% ---------- OPTIMAL TRAJECTORY (k-NN graph) ---------------------------
p_init  =  [-100 -100];         % start (physical coordinate)
p_final = [ -100  100];         % destination

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

% -------- NoT : No orientation Tracking --------

q_rot=0; 
q_omega=0;
q_sv=2*1e-12;
q_p=6;
q_h=10;
q_v=1;

sv_init = 0;
sv_max  = 7*1e7;

[controls_NoT, state_NoT, time_NoT, data_NoT, solve_time_NoT] = ...
    Simulation_Main(settings,opt,N,Ns,sv_init,sv_max,q_sv,q_p,q_h,q_v,q_rot,q_omega, ...
                    h_UAV,pathXY_ProxyUtility,Tf_init,pA,pR,freq,p_bar_User,M_ele);

% -------- HoT : Horizontal orientation Tracking--------
q_rot=10; 
q_omega=30;
q_sv=2*1e-12;
q_p=6;
q_h=10;
q_v=1;

sv_init = 0;
sv_max  = 7*1e7;

[controls_HoT, state_HoT, time_HoT, data_HoT, solve_time_HoT] = ...
    Simulation_Main(settings,opt,N,Ns,sv_init,sv_max,q_sv,q_p,q_h,q_v,q_rot,q_omega, ...
                    h_UAV,pathXY_ProxyUtility,Tf_init,pA,pR,freq,p_bar_User,M_ele);


% -------- SL : Straight Line Tracking (Third Benchmark) --------
% 1. Create a straight line using linspace to match exactly the un-padded length (Ns - N)
Ns_base = Ns - N;
x_sl = linspace(p_init(1), p_final(1), Ns_base)';
y_sl = linspace(p_init(2), p_final(2), Ns_base)';
pathXY_Straight = [x_sl, y_sl];

% 2. Enforce the final position by padding N extra elements at the end
pathXY_Straight = [pathXY_Straight; repmat(p_final, N, 1)];

% Weights setup (Copied from HoT, adjust if you prefer NoT's setup)
q_rot=10; 
q_omega=30;
q_sv=2*1e-12;
q_p=6;
q_h=10;
q_v=1;

sv_init = 0;
sv_max  = 7*1e7;

% 3. Call NMPC on the straight line reference
[controls_SL, state_SL, time_SL, data_SL, solve_time_SL] = ...
    Simulation_Main(settings,opt,N,Ns,sv_init,sv_max,q_sv,q_p,q_h,q_v,q_rot,q_omega, ...
                    h_UAV,pathXY_Straight,Tf_init,pA,pR,freq,p_bar_User,M_ele);




% -------- HOV: PSCA for localisation, beamforming and phase shift optimization (Fourth Benchmark) --------
pU_init_HOV=[p_bar_User(1),p_bar_User(2),h_UAV];
[pU_opt_HOV, theta_opt_HOV, f_opt_HOV, obj_evolution_HOV] = ...
    PSCA_Optimal_Position(settings, pU_init_HOV);

save("Main.mat");

Draw_Main;
