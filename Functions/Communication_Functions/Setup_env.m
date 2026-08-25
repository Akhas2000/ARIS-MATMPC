function [pA, pR, pK] = Setup_env(N, K, M_size, freq, seed)
%SETUP_ENV Generate antenna, RIS, and user positions.
%
% Inputs:
%   N      - Number of antenna elements
%   K      - Number of users
%   M_size - [M_H, M_V], horizontal and vertical RIS dimensions
%   freq   - Carrier frequency in Hz
%   seed   - Optional random seed. If omitted or empty, rng('shuffle') is used.
%
% Outputs:
%   pA - Antenna element positions, pA(:,n)
%   pR - RIS element positions, pR(:,m)
%   pK - User positions, pK(:,k)

% Configure the random-number generator
if nargin < 5 || isempty(seed)
    rng('shuffle');
else
    rng(seed);
end

% User deployment area
x_min = 100;
x_max = 150;
y_min = -30;
y_max = 30;

% Base-station height
h_BS = 68;

% Horizontal and vertical numbers of RIS elements
M_H = M_size(1);
M_V = M_size(2);
M   = M_H * M_V;

% Wavelength
lambda = 3e8 / freq;

% Antenna and RIS element spacing
d_HA = lambda / 2;
d_VA = lambda / 2;
d_HR = lambda / 2;
d_VR = lambda / 2;

% Initialize antenna, RIS, and user positions
pA = zeros(3, N);
pR = zeros(3, M);
pK = zeros(3, K);

% Antenna positions
for n = 1:N
    pA(1,n) = d_HA;
    pA(3,n) = (n - 1) * d_VA;
end

% RIS positions
for m = 1:M
    m_V = floor((m - 1) / M_H);
    m_H = mod(m - 1, M_H);

    pR(1,m) = d_VR * m_V;
    pR(2,m) = d_HR * m_H;
end

% User positions
for k = 1:K
    pK(1,k) = x_min + (x_max - x_min) * rand;
    pK(2,k) = y_min + (y_max - y_min) * rand;
    pK(3,k) = -h_BS;
end

end