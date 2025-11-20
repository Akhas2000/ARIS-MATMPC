function [totalRate, ratesVec] = computeTotalRateAlongPath( ...
    pathXY, h_UAV, pA, pR, pK, R, Power, Bandwidth, freq, beta_B, beta_R)
% COMPUTETOTALRATEALONGPATH  Computes the total sum-rate along a UAV path.
%
% INPUTS:
%   pathXY    : L×2 matrix of 2D path coordinates [x y]
%   h_UAV     : scalar, fixed UAV height
%   pA, pR, pK: environment positions (BS, RIS, Users)
%   R         : 3x3 rotation matrix (e.g., eye(3))
%   Power     : 1×K user Tx power vector
%   Bandwidth : 1×K bandwidth vector
%   freq      : carrier frequency [Hz]
%   beta_B, beta_R : path loss constants (linear)
%
% OUTPUTS:
%   totalRate : scalar, accumulated rate [bps/Hz]
%   ratesVec  : L×1 vector of sum-rate per waypoint

    L = size(pathXY,1);
    ratesVec = zeros(L,1);  % preallocate

    p_bar_User = mean(pK,2)';  % centroid of users

    for l = 1:L
        pU = [pathXY(l,:), h_UAV];  % [x y z]

        f  = array_response_BS(pA, pU, freq);  
        f  = f / norm(f);

        P_A_rx_RIS = phase_array_response_RIS(pR, pU, [], R, freq);
        P_A_tx_RIS = phase_array_response_RIS(pR, pU, p_bar_User, R, freq);
        theta      = P_A_tx_RIS - P_A_rx_RIS;

        Rates = Rates_No_Complex(pA, pR, pU, pK, R, theta, ...
                                 f, Power, Bandwidth, freq, ...
                                 beta_B, beta_R);
        ratesVec(l) = sum(Rates);  % sum over all users
    end

    totalRate = sum(ratesVec);  % global rate along the path
end
