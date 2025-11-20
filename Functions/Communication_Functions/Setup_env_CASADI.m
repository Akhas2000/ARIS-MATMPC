function [pA, pR, pK] = Setup_env_CASADI(N, K, M_size, freq)
    % Setup_env_CASADI
    % CASADI-compatible setup for antenna, RIS, and user positions
    % with random user locations converted to SX format.
    %
    % Inputs:
    %   N      : Number of antenna elements
    %   K      : Number of users
    %   M_size : [M_H, M_V] = horizontal × vertical RIS elements
    %   freq   : Carrier frequency (Hz)
    %
    % Outputs:
    %   pA     : 3 x N CasADi SX matrix - antenna element positions
    %   pR     : 3 x M CasADi SX matrix - RIS element positions
    %   pK     : 3 x K CasADi SX matrix - user positions (randomized)

    import casadi.*

    % --- Area limits for user distribution
    x_min = 140;
    x_max = 150;
    y_min = -10;
    y_max = 10;

    % --- Base station height
    h_BS = 68;

    % --- RIS configuration
    M_H = M_size(1);
    M_V = M_size(2);
    M = M_H * M_V;

    % --- Wavelength and element spacing
    lambda = 3e8 / freq;
    d_HA = lambda / 2;  % antenna horizontal spacing
    d_VA = lambda / 2;  % antenna vertical spacing
    d_HR = lambda / 2;  % RIS horizontal spacing
    d_VR = lambda / 2;  % RIS vertical spacing

    % --- Initialize symbolic matrices
    pA = SX(zeros(3, N));  % Antenna positions
    pR = SX(zeros(3, M));  % RIS positions

    % --- Generate user positions numerically
    pK_numeric = zeros(3, K);
    for k = 1:K
        pK_numeric(1, k) = x_min + (x_max - x_min) * rand; % x
        pK_numeric(2, k) = y_min + (y_max - y_min) * rand; % y
        pK_numeric(3, k) = -h_BS;                          % z
    end

    % --- Convert numeric user positions to SX
    pK = SX(pK_numeric);

    % --- Fill antenna positions (vertical stack)
    for n = 1:N
        pA(1, n) = d_HA;                % x
        pA(2, n) = 0;                   % y
        pA(3, n) = (n - 1) * d_VA;      % z
    end

    % --- Fill RIS positions (planar array)
    for m = 1:M
        m_V = floor((m - 1) / M_V);
        m_H = mod((m - 1), M_H);
        pR(1, m) = d_VR * m_V;  % x
        pR(2, m) = d_HR * m_H;  % y
        pR(3, m) = 0;           % z
    end
end
