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


N  = 30;             % No. of shooting points
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




%% Simulation Duration
Tf_init =26;  % simulation time
T_stabilization=4;
Tf=Tf_init+T_stabilization;

%% Reference Trajectory Generation

%% ---------- AREA & RANDOM CLOUD SIZE ----------------------------------
xmin = -200;  xmax =  200;            % [m] rectangle in x
ymin = -200;  ymax =  200;            % [m] rectangle in y
h_UAV = 100;                          % [m] fixed altitude
Nrand = 30000;                        % << huge number of random nodes

R = eye(3);                           % rotation (identity)
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
% seed_Ref_gen=5;
% rng(seed_Ref_gen);
x_vals = xmin + (xmax-xmin)*rand(1,Nrand);   % 1×Nrand
y_vals = ymin + (ymax-ymin)*rand(1,Nrand);   % 1×Nrand
R_mat   = zeros(1,Nrand);                     % Σ-rate per node
Proxy_Utility_mat   = zeros(1,Nrand);                     % Σ-rate per node

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
p_init  =  [-100 -100];         % start (physical coordinate)
p_final = [ -100   100];      % destination


[pathXY_ProxyUtility] = optimalTrajectoryPU_Astar( ...
                           Proxy_Utility_mat, x_vals, y_vals, ...
                           p_init, p_final, ...
                           'k', 8);            % 8-nearest neighbours


Ns = Tf_init/Ts_st;                           % pick any integer ≥ size(pathXY,1)
pathXY_ProxyUtility= densifyToNs(pathXY_ProxyUtility, Ns);

N_stabilization=N+T_stabilization/Ts_st;
X_end=pathXY_ProxyUtility(end,1);
Y_end=pathXY_ProxyUtility(end,2);
for p_iter=0:N_stabilization
    pathXY_ProxyUtility=[pathXY_ProxyUtility;X_end,Y_end];
end
pathXY_ProxyUtility = smoothPath(pathXY_ProxyUtility, 150);
Ns=size(pathXY_ProxyUtility,1);



% -------- NoT : No orientation Tracking --------
q_eul   = [0;0;0];
q_omega = 0.03;
q_p     = 1.4;
q_v     = 0.2;
q_sv    = 0.5e-11;

[controls_NoT, state_NoT, time_NoT, data_NoT] = ...
    Simulation_Main(settings,opt,N,Ns,q_sv,q_p,q_v,q_eul,q_omega, ...
                    h_UAV,pathXY_ProxyUtility,Tf_init,T_stabilization);

% -------- HoT : Horizontal orientation Tracking --------
q_eul   = [8;8;8];
q_omega = 0.03;
q_p     = 1.4;
q_v     = 0.2;
q_sv    = 0.5e-11;

[controls_HoT, state_HoT, time_HoT, data_HoT] = ...
    Simulation_Main(settings,opt,N,Ns,q_sv,q_p,q_v,q_eul,q_omega, ...
                    h_UAV,pathXY_ProxyUtility,Tf_init,T_stabilization);

save("HoT_NoT_uMRAV_Both.mat")
Draw_Main;