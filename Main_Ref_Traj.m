clear all; clear mex; close all; clc;

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

%% Reference Trajectory Generation

% Simulation Duration
Tf_init = 45;  % simulation time

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

Rmat              = zeros(1,Nrand);          % Σ-rate per node
Proxy_Utility_2   = zeros(1,Nrand);          % Proxy utility p=2
Proxy_Utility_5   = zeros(1,Nrand);          % Proxy utility p=5
Proxy_Utility_8   = zeros(1,Nrand);          % Proxy utility p=8

% Start & Destination coordinates
p_init  = [-100, -100];         
p_final = [-100,  100];         

x_vals(1) = p_init(1); x_vals(Nrand) = p_final(1);
y_vals(1) = p_init(2); y_vals(Nrand) = p_final(2);

disp('Calculating Rates and Utilities for cloud nodes... Please wait.');
% ---------- RATE & UTILITY CALCULATION --------------------------------
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

    % Base proxy utility calculation
    norm_diff = norm(pU - p_bar_User);
    norm_self = norm(pU);
    base_PU   = norm_diff * norm_self;
    
    Proxy_Utility_2(n) = base_PU^2;
    Proxy_Utility_5(n) = base_PU^5;
    Proxy_Utility_8(n) = base_PU^8;
end
disp('Cloud evaluation complete.');

%% ---------- OPTIMAL TRAJECTORIES (k-NN graph) ---------------------------
disp('Generating optimal trajectories...');
k_neighbors = 8;

% 1. Maximize Rate (using the provided Rate-maximizing function)
[pathXY_Rate] = optimalTrajectoryRandomKNN(Rmat, x_vals, y_vals, p_init, p_final, 'k', k_neighbors);

% 2. Minimize Proxy Utility p=2, 5, 8 using optimalTrajectoryPU
[pathXY_PU2] = optimalTrajectoryPU(Proxy_Utility_2, x_vals, y_vals, p_init, p_final, 'k', k_neighbors);

[pathXY_PU5] = optimalTrajectoryPU(Proxy_Utility_5, x_vals, y_vals, p_init, p_final, 'k', k_neighbors);

[pathXY_PU8] = optimalTrajectoryPU(Proxy_Utility_8, x_vals, y_vals, p_init, p_final, 'k', k_neighbors);

disp('Trajectories generated successfully.');
%% ------------------------------------------
%  Figure: Reference Trajectories Comparison
%  ------------------------------------------
figure;

scatter(x_vals, y_vals, 12, Rmat, 'filled');      % colour = Σ-rate
axis equal;
grid on;
colorbar;
xlabel('x (m)');
ylabel('y (m)');
hold on;

% BS
p_bs = plot(0,0,'rs','MarkerSize',10,'MarkerFaceColor','r');

% Users
p_us = plot(pK(1,:),pK(2,:),'ro','MarkerSize',5,'MarkerFaceColor','r');

% User barycenter
p_bar = plot(p_bar_User(1),p_bar_User(2), ...
             'kp','MarkerSize',10,'MarkerFaceColor','m');

% Reference trajectories
p_rate = plot(pathXY_Rate(:,1),pathXY_Rate(:,2), ...
              'k-','LineWidth',2);

p_pu2 = plot(pathXY_PU2(:,1),pathXY_PU2(:,2), ...
             'r--','LineWidth',2);

p_pu5 = plot(pathXY_PU5(:,1),pathXY_PU5(:,2), ...
             'g--','LineWidth',2);

p_pu8 = plot(pathXY_PU8(:,1),pathXY_PU8(:,2), ...
             'b--','LineWidth',2);

% Start and finish
plot(p_init(1),p_init(2),'rs','MarkerFaceColor','r', ...
     'HandleVisibility','off');

plot(p_final(1),p_final(2),'rd','MarkerFaceColor','r', ...
     'HandleVisibility','off');

legend({'Rate samples', ...
        'BS', ...
        'Users', ...
        'User barycenter', ...
        'Optimal Rate Path', ...
        'Proxy Utility (p=2)', ...
        'Proxy Utility (p=5)', ...
        'Proxy Utility (p=8)'}, ...
       'Location','northoutside', ...
       'NumColumns',2);

xlim([xmin xmax]);
ylim([ymin ymax]);

hold off;


save("Main_Ref_Traj.mat");
