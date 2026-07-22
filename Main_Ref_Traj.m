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

%% Reference Trajectory Generation

% Simulation Duration
Tf_init = 80;  % simulation time

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
p_bs = plot(0, 0,'rs','MarkerSize', MS_big,'MarkerFaceColor','r');
% Users
p_us = plot(pK(1,:),pK(2,:),'ro','MarkerSize', MS,'MarkerFaceColor','r');
% User barycenter
p_bar = plot(p_bar_User(1),p_bar_User(2), ...
'kp','MarkerSize', MS_big,'MarkerFaceColor','m');
% Reference trajectories
p_rate = plot(pathXY_Rate(:,1),pathXY_Rate(:,2), ...
'k-','LineWidth', LWth);
p_pu2 = plot(pathXY_PU2(:,1),pathXY_PU2(:,2), ...
'r--','LineWidth', LWth);
p_pu5 = plot(pathXY_PU5(:,1),pathXY_PU5(:,2), ...
'g--','LineWidth', LWth);
p_pu8 = plot(pathXY_PU8(:,1),pathXY_PU8(:,2), ...
'b--','LineWidth', LWth);
% Start and finish
plot(p_init(1),p_init(2),'rs','MarkerFaceColor','r', ...
'HandleVisibility','off');
plot(p_final(1),p_final(2),'rd','MarkerFaceColor','r', ...
'HandleVisibility','off');
legend({'Rate samples', ...
'BS', ...
'Users', ...
'User barycenter', ...
'RRG', ...
'PURG (\gamma=2)', ...
'PURG (\gamma=5)', ...
'PURG (\gamma=8)'}, ...
'Location','northoutside', ...
'NumColumns',2);
% --- Match reference PDF axis limits ---
lim_xmin = xmin; lim_xmax = xmax;
lim_ymin = ymin; lim_ymax = ymax;
xlim([lim_xmin lim_xmax]);
ylim([lim_ymin lim_ymax]);
cb = colorbar;
cb.Label.String = 'Total network rate (bit/s)';
cb.Label.FontSize = FSL;
%set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', AXLW);
hold off;

% %% =====================================================
% % Densify trajectories to Ns samples
% %% =====================================================
% Tf = Tf_init;
% Ns = round(Tf/Ts);
% trajList = {pathXY_Rate, pathXY_PU2, pathXY_PU5, pathXY_PU8};
% figure;
% hold on; grid on;
% styles = {'k-','r--','g-.','b:'};
% time = (0:Ns-1)*Ts;
% for t = 1:length(trajList)
%     path = trajList{t};
% % Original parameter
%     s0 = linspace(0,1,size(path,1));
% % New parameter
%     s = linspace(0,1,Ns);
% % Densified trajectory
%     pathDense(:,1) = interp1(s0,path(:,1),s,'pchip');
%     pathDense(:,2) = interp1(s0,path(:,2),s,'pchip');
%     rate = zeros(Ns,1);
% for k = 1:Ns
%         pU = [pathDense(k,1), pathDense(k,2), h_UAV];
%         f_phases = array_response_phases_BS(pA,pU,freq);
%         P_A_rx_RIS = phase_array_response_RIS(pR,pU,[],R,freq);
%         P_A_tx_RIS = phase_array_response_RIS(pR,pU,p_bar_User,R,freq);
%         theta = P_A_tx_RIS - P_A_rx_RIS;
%         Rates = Rates_No_Complex_phase(...
%             pA,pR,pU,pK,R,theta,...
%             f_phases,Power,Bandwidth,...
%             freq,beta_B,beta_R);
%         rate(k) = sum(Rates);
% end
%     cumulativeRate = cumsum(rate);
%     plot(time,cumulativeRate,...
%         styles{t},'LineWidth', LWth);
% end
% xlabel('Time (s)');
% ylabel('Cumulative Data Rate');
% legend('RRG',...
% 'PURG (\gamma=2)',...
% 'PURG (\gamma=5)',...
% 'PURG (\gamma=8)',...
% 'Location','northwest');
% xlim([0 Tf]);
% set(gca, 'FontSize', FS, 'LineWidth', AXLW);
% %set(gcf, 'Units', 'inches', 'Position', FIGSIZE);   % physical size
% set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
% set(gca, 'FontSize', FS, 'LineWidth', AXLW);


%% =====================================================
% Densify trajectories to Ns samples
%% =====================================================
Tf = Tf_init;
Ns = round(Tf/Ts);

trajList = {pathXY_Rate, pathXY_PU2, pathXY_PU5, pathXY_PU8};
styles = {'k-','r--','g-.','b:'};
time = (0:Ns-1)*Ts;

% First pass: compute cumulative rates and global maximum
cumRates = cell(length(trajList),1);
globalMax = 0;

for t = 1:length(trajList)

    path = trajList{t};

    s0 = linspace(0,1,size(path,1));
    s  = linspace(0,1,Ns);

    pathDense(:,1) = interp1(s0,path(:,1),s,'pchip');
    pathDense(:,2) = interp1(s0,path(:,2),s,'pchip');

    rate = zeros(Ns,1);

    for k = 1:Ns
        pU = [pathDense(k,1), pathDense(k,2), h_UAV];

        f_phases = array_response_phases_BS(pA,pU,freq);
        P_A_rx_RIS = phase_array_response_RIS(pR,pU,[],R,freq);
        P_A_tx_RIS = phase_array_response_RIS(pR,pU,p_bar_User,R,freq);

        theta = P_A_tx_RIS - P_A_rx_RIS;

        Rates = Rates_No_Complex_phase(...
            pA,pR,pU,pK,R,theta,...
            f_phases,Power,Bandwidth,...
            freq,beta_B,beta_R);

        rate(k) = sum(Rates);
    end

    cumRates{t} = cumsum(rate);
    globalMax = max(globalMax, max(cumRates{t}));
end

% Second pass: plot normalized curves
figure;
hold on;
grid on;

for t = 1:length(trajList)
    plot(time, cumRates{t}/globalMax, ...
        styles{t}, 'LineWidth', LWth);
end

xlabel('Time (s)');
ylabel('Normalized Cumulative Data Rate');
ylim([0 1.1]);

legend('RRG', ...
       'PURG (\gamma=2)', ...
       'PURG (\gamma=5)', ...
       'PURG (\gamma=8)', ...
       'Location','northwest');

xlim([0 Tf]);

set(findall(gcf,'-property','FontSize'), 'FontSize', FS);
set(gca, 'FontSize', FS, 'LineWidth', AXLW);


save("Main_Ref_Traj.mat");
