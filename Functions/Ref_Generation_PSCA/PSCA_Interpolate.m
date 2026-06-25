
% function [Ref_Traj_ZOH, Ref_Theta_ZOH, Ref_Beam_ZOH, t_NMPC_ext] = ...
%     PSCA_Interpolate(P_bar, Theta_bar, F_phases_bar, p_init, p_fin, ...
%                       T_total, Ts_st, N_horizon, smooth_win)
% %PSCA_INTERPOLATE Interpolates PSCA-optimized trajectory/RIS/beam onto the
% %MATMPC ZOH grid and appends a terminal hold for the MPC horizon.
% %
% % INPUTS:
% %   P_bar         - 3 x N_steps optimized 3D trajectory
% %   Theta_bar     - M_ele x N_steps optimized RIS phase shifts
% %   F_phases_bar  - N_A x N_steps optimized BS beamforming phases
% %   p_init        - 3x1 initial position (pins trajectory boundary)
% %   p_fin         - 3x1 final position (pins trajectory boundary)
% %   T_total       - Total PSCA simulation time
% %   Ts_st         - MPC shooting interval
% %   N_horizon     - MPC horizon length (number of terminal samples held)
% %   smooth_win    - (optional) moving-average window for trajectory
% %                   pre-smoothing, odd integer. Default: 5.
% %
% % OUTPUTS:
% %   Ref_Traj_ZOH  - 3 x (Ns+N_horizon) trajectory: smoothly densified to
% %                   half-size, then zero-order-held up to full size
% %   Ref_Theta_ZOH - M_ele x (Ns+N_horizon) RIS phases: same treatment
% %   Ref_Beam_ZOH  - N_A x (Ns+N_horizon) beamforming phases: same treatment
% %   t_NMPC_ext    - 1 x (Ns+N_horizon) extended time vector
% 
%     if nargin < 9 || isempty(smooth_win)
%         smooth_win = 5;
%     end
% 
%     N_steps = size(P_bar, 2);
% 
%     %% Interpolation to MATMPC Setup
%     Tf_init = T_total;
%     Ns = floor(Tf_init/Ts_st) + 1;
%     t_PSCA = linspace(0, T_total, N_steps);
%     t_NMPC = (0:Ns-1) * Ts_st;
% 
%     % Shared half-size grid used by trajectory, theta, and beam
%     N_half = max(2, round(Ns/2));
%     t_half = linspace(0, T_total, N_half);
% 
%     % --- Trajectory: pre-smooth, densify to half-size, then ZOH ----------
%     P_smooth = movmean(P_bar, smooth_win, 2);
%     P_smooth(:,1)   = p_init;     % restore exact boundary conditions
%     P_smooth(:,end) = p_fin;
% 
%     P_dense = interp1(t_PSCA, P_smooth.', t_half, 'makima', 'extrap').';   % Step 1: smooth densify
%     Ref_Traj_ZOH = interp1(t_half, P_dense.', t_NMPC, 'previous', 'extrap').';  % Step 2: ZOH
% 
%     % --- RIS phases: unwrap, smooth densify to half-size, then ZOH -------
%     Theta_unwrapped = unwrap(Theta_bar.', [], 1);                                   % N_steps x M_ele
%     Theta_dense     = interp1(t_PSCA, Theta_unwrapped, t_half, 'makima', 'extrap'); % Step 1: smooth densify
%     Ref_Theta_ZOH   = interp1(t_half, Theta_dense, t_NMPC, 'previous', 'extrap').'; % Step 2: ZOH
%     Ref_Theta_ZOH   = mod(Ref_Theta_ZOH, 2*pi);                                     % re-wrap
% 
%     % --- BS beamforming phases: unwrap, smooth densify, then ZOH ----------
%     Beam_unwrapped = unwrap(F_phases_bar.', [], 1);                                 % N_steps x N_A
%     Beam_dense     = interp1(t_PSCA, Beam_unwrapped, t_half, 'makima', 'extrap');   % Step 1: smooth densify
%     Ref_Beam_ZOH   = interp1(t_half, Beam_dense, t_NMPC, 'previous', 'extrap').';   % Step 2: ZOH
%     Ref_Beam_ZOH   = mod(Ref_Beam_ZOH, 2*pi);                                       % re-wrap
% 
%     %% Enforce terminal reference over last N samples for MPC
%     Nnew = N_horizon;
%     Ref_Traj_ZOH  = [Ref_Traj_ZOH,  repmat(Ref_Traj_ZOH(:,end), 1, Nnew)];
%     Ref_Theta_ZOH = [Ref_Theta_ZOH, repmat(Ref_Theta_ZOH(:,end), 1, Nnew)];
%     Ref_Beam_ZOH  = [Ref_Beam_ZOH,  repmat(Ref_Beam_ZOH(:,end), 1, Nnew)];
%     t_NMPC_ext = [t_NMPC, t_NMPC(end) + Ts_st*(1:Nnew)];
% 
% end
% 
% 
% 
% 
% 
% 












% function [Ref_Traj_ZOH, Ref_Theta_ZOH, Ref_Beam_ZOH, t_NMPC_ext] = ...
%     PSCA_Interpolate(P_bar, Theta_bar, F_phases_bar, p_init, p_fin, ...
%                       T_total, Ts_st, N_horizon, smooth_win)
% %PSCA_INTERPOLATE Interpolates PSCA-optimized trajectory/RIS/beam onto the
% %MATMPC ZOH grid and appends a terminal hold for the MPC horizon.
% %
% % INPUTS:
% %   P_bar         - 3 x N_steps optimized 3D trajectory
% %   Theta_bar     - M_ele x N_steps optimized RIS phase shifts
% %   F_phases_bar  - N_A x N_steps optimized BS beamforming phases
% %   p_init        - 3x1 initial position (pins trajectory boundary)
% %   p_fin         - 3x1 final position (pins trajectory boundary)
% %   T_total       - Total PSCA simulation time
% %   Ts_st         - MPC shooting interval
% %   N_horizon     - MPC horizon length (number of terminal samples held)
% %   smooth_win    - (optional) moving-average window for trajectory
% %                   pre-smoothing, odd integer. Default: 5.
% %
% % OUTPUTS:
% %   Ref_Traj_ZOH  - 3 x (Ns+N_horizon) trajectory: smoothly densified to
% %                   half-size, then zero-order-held up to full size
% %   Ref_Theta_ZOH - M_ele x (Ns+N_horizon) RIS phases, pure ZOH (no interp)
% %   Ref_Beam_ZOH  - N_A x (Ns+N_horizon) beamforming phases, pure ZOH
% %   t_NMPC_ext    - 1 x (Ns+N_horizon) extended time vector
% 
%     if nargin < 9 || isempty(smooth_win)
%         smooth_win = 5;
%     end
% 
%     N_steps = size(P_bar, 2);
% 
%     %% Interpolation to MATMPC Setup
%     Tf_init = T_total;
%     Ns = floor(Tf_init/Ts_st) + 1;
%     t_PSCA = linspace(0, T_total, N_steps);
%     t_NMPC = (0:Ns-1) * Ts_st;
% 
%     % --- Trajectory: pre-smooth, densify to half-size, then ZOH ----------
%     P_smooth = movmean(P_bar, smooth_win, 2);
%     P_smooth(:,1)   = p_init;     % restore exact boundary conditions
%     P_smooth(:,end) = p_fin;
% 
%     % Step 1: smoothly densify to half the final NMPC grid size
%     N_half = max(2, round(Ns/2));
%     t_half = linspace(0, T_total, N_half);
%     P_dense = interp1(t_PSCA, P_smooth.', t_half, 'makima', 'extrap').';
% 
%     % Step 2: zero-order hold from the half-density curve to full size
%     Ref_Traj_ZOH = interp1(t_half, P_dense.', t_NMPC, 'previous', 'extrap').';
% 
%     % --- RIS phases: pure zero-order hold (no unwrap, no interpolation) --
%     Ref_Theta_ZOH = interp1(t_PSCA, Theta_bar.', t_NMPC, 'previous', 'extrap').';
% 
%     % --- BS beamforming phases: pure zero-order hold ----------------------
%     Ref_Beam_ZOH = interp1(t_PSCA, F_phases_bar.', t_NMPC, 'previous', 'extrap').';
% 
%     %% Enforce terminal reference over last N samples for MPC
%     Nnew = N_horizon;
%     Ref_Traj_ZOH  = [Ref_Traj_ZOH,  repmat(Ref_Traj_ZOH(:,end), 1, Nnew)];
%     Ref_Theta_ZOH = [Ref_Theta_ZOH, repmat(Ref_Theta_ZOH(:,end), 1, Nnew)];
%     Ref_Beam_ZOH  = [Ref_Beam_ZOH,  repmat(Ref_Beam_ZOH(:,end), 1, Nnew)];
%     t_NMPC_ext = [t_NMPC, t_NMPC(end) + Ts_st*(1:Nnew)];
% 
% end
% 
% 
% 
% 
% 
% 
% 
% 
% 


% function [Ref_Traj_ZOH, Ref_Theta_ZOH, Ref_Beam_ZOH, t_NMPC_ext] = ...
%     PSCA_Interpolate(P_bar, Theta_bar, F_phases_bar, p_init, p_fin, ...
%                       T_total, Ts_st, N_horizon, smooth_win)
% %PSCA_INTERPOLATE Interpolates PSCA-optimized trajectory/RIS/beam onto the
% %MATMPC ZOH grid and appends a terminal hold for the MPC horizon.
% %
% % INPUTS:
% %   P_bar         - 3 x N_steps optimized 3D trajectory
% %   Theta_bar     - M_ele x N_steps optimized RIS phase shifts
% %   F_phases_bar  - N_A x N_steps optimized BS beamforming phases
% %   p_init        - 3x1 initial position (pins trajectory boundary)
% %   p_fin         - 3x1 final position (pins trajectory boundary)
% %   T_total       - Total PSCA simulation time
% %   Ts_st         - MPC shooting interval
% %   N_horizon     - MPC horizon length (number of terminal samples held)
% %   smooth_win    - (optional) moving-average window for trajectory
% %                   pre-smoothing, odd integer. Default: 5.
% %
% % OUTPUTS:
% %   Ref_Traj_ZOH  - 3 x (Ns+N_horizon) trajectory: smoothly densified to
% %                   half-size, then zero-order-held up to full size
% %   Ref_Theta_ZOH - M_ele x (Ns+N_horizon) interpolated RIS phases
% %   Ref_Beam_ZOH  - N_A x (Ns+N_horizon) interpolated beamforming phases
% %   t_NMPC_ext    - 1 x (Ns+N_horizon) extended time vector
% 
%     if nargin < 9 || isempty(smooth_win)
%         smooth_win = 5;
%     end
% 
%     N_steps = size(P_bar, 2);
% 
%     %% Interpolation to MATMPC Setup
%     Tf_init = T_total;
%     Ns = floor(Tf_init/Ts_st) + 1;
%     t_PSCA = linspace(0, T_total, N_steps);
%     t_NMPC = (0:Ns-1) * Ts_st;
% 
%     % --- Trajectory: pre-smooth, densify to half-size, then ZOH ----------
%     P_smooth = movmean(P_bar, smooth_win, 2);
%     P_smooth(:,1)   = p_init;     % restore exact boundary conditions
%     P_smooth(:,end) = p_fin;
% 
%     % Step 1: smoothly densify to half the final NMPC grid size
%     N_half = max(2, round(Ns/2));
%     t_half = linspace(0, T_total, N_half);
%     P_dense = interp1(t_PSCA, P_smooth.', t_half, 'makima', 'extrap').';
% 
%     % Step 2: zero-order hold from the half-density curve to full size
%     Ref_Traj_ZOH = interp1(t_half, P_dense.', t_NMPC, 'previous', 'extrap').';
% 
%     % --- RIS phases: plain interpolation (unwrap -> interp -> re-wrap) ---
%     Theta_unwrapped = unwrap(Theta_bar.', [], 1);
%     Ref_Theta_ZOH   = interp1(t_PSCA, Theta_unwrapped, t_NMPC, 'pchip', 'extrap').';
%     Ref_Theta_ZOH   = mod(Ref_Theta_ZOH, 2*pi);
% 
%     % --- BS beamforming phases: plain interpolation -----------------------
%     Beam_unwrapped = unwrap(F_phases_bar.', [], 1);
%     Ref_Beam_ZOH   = interp1(t_PSCA, Beam_unwrapped, t_NMPC, 'pchip', 'extrap').';
%     Ref_Beam_ZOH   = mod(Ref_Beam_ZOH, 2*pi);
% 
%     %% Enforce terminal reference over last N samples for MPC
%     Nnew = N_horizon;
%     Ref_Traj_ZOH  = [Ref_Traj_ZOH,  repmat(Ref_Traj_ZOH(:,end), 1, Nnew)];
%     Ref_Theta_ZOH = [Ref_Theta_ZOH, repmat(Ref_Theta_ZOH(:,end), 1, Nnew)];
%     Ref_Beam_ZOH  = [Ref_Beam_ZOH,  repmat(Ref_Beam_ZOH(:,end), 1, Nnew)];
%     t_NMPC_ext = [t_NMPC, t_NMPC(end) + Ts_st*(1:Nnew)];
% 
% end
% 
% 



function [Ref_Traj_ZOH, Ref_Theta_ZOH, Ref_Beam_ZOH, t_NMPC_ext] = ...
    PSCA_Interpolate(P_bar, Theta_bar, F_phases_bar, p_init, p_fin, ...
                      T_total, Ts_st, N_horizon, smooth_win)
%PSCA_INTERPOLATE Interpolates PSCA-optimized trajectory/RIS/beam onto the
%MATMPC ZOH grid and appends a terminal hold for the MPC horizon.
%
% INPUTS:
%   P_bar         - 3 x N_steps optimized 3D trajectory
%   Theta_bar     - M_ele x N_steps optimized RIS phase shifts
%   F_phases_bar  - N_A x N_steps optimized BS beamforming phases
%   p_init        - 3x1 initial position (pins trajectory boundary)
%   p_fin         - 3x1 final position (pins trajectory boundary)
%   T_total       - Total PSCA simulation time
%   Ts_st         - MPC shooting interval
%   N_horizon     - MPC horizon length (number of terminal samples held)
%   smooth_win    - (optional) moving-average window for trajectory
%                   pre-smoothing, odd integer. Default: 5.
%
% OUTPUTS:
%   Ref_Traj_ZOH  - 3 x (Ns+N_horizon) smoothed/interpolated trajectory
%   Ref_Theta_ZOH - M_ele x (Ns+N_horizon) interpolated RIS phases
%   Ref_Beam_ZOH  - N_A x (Ns+N_horizon) interpolated beamforming phases
%   t_NMPC_ext    - 1 x (Ns+N_horizon) extended time vector

    if nargin < 9 || isempty(smooth_win)
        smooth_win = 5;
    end

    N_steps = size(P_bar, 2);

    %% Interpolation to MATMPC Setup
    Tf_init = T_total;
    Ns = floor(Tf_init/Ts_st) + 1;
    t_PSCA = linspace(0, T_total, N_steps);
    t_NMPC = (0:Ns-1) * Ts_st;

    % --- Trajectory: light pre-smoothing + smooth interpolation ----------
    P_smooth = movmean(P_bar, smooth_win, 2);
    P_smooth(:,1)   = p_init;     % restore exact boundary conditions
    P_smooth(:,end) = p_fin;
    Ref_Traj_ZOH = interp1(t_PSCA, P_smooth.', t_NMPC, 'makima', 'extrap').';

    % --- RIS phases: plain interpolation (unwrap -> interp -> re-wrap) ---
    Theta_unwrapped = unwrap(Theta_bar.', [], 1);
    Ref_Theta_ZOH   = interp1(t_PSCA, Theta_unwrapped, t_NMPC, 'pchip', 'extrap').';
    Ref_Theta_ZOH   = mod(Ref_Theta_ZOH, 2*pi);

    % --- BS beamforming phases: plain interpolation -----------------------
    Beam_unwrapped = unwrap(F_phases_bar.', [], 1);
    Ref_Beam_ZOH   = interp1(t_PSCA, Beam_unwrapped, t_NMPC, 'pchip', 'extrap').';
    Ref_Beam_ZOH   = mod(Ref_Beam_ZOH, 2*pi);

    %% Enforce terminal reference over last N samples for MPC
    Nnew = N_horizon;
    Ref_Traj_ZOH  = [Ref_Traj_ZOH,  repmat(Ref_Traj_ZOH(:,end), 1, Nnew)];
    Ref_Theta_ZOH = [Ref_Theta_ZOH, repmat(Ref_Theta_ZOH(:,end), 1, Nnew)];
    Ref_Beam_ZOH  = [Ref_Beam_ZOH,  repmat(Ref_Beam_ZOH(:,end), 1, Nnew)];
    t_NMPC_ext = [t_NMPC, t_NMPC(end) + Ts_st*(1:Nnew)];

end