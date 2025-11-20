function F = compute_aperture_gain_from_rotation(pU, p_user, R)
% Compute effective aperture gain F = cos(phi1) * cos(phi2)
% Inputs:
%   pU      : 3x1 UAV position vector [qx; qy; qz]
%   p_user  : 3x1 user position in global frame
%   R       : 3x3 rotation matrix (global to UAV local frame)
% Output:
%   F       : Effective aperture gain (between 0 and 1)

% Ensure correct vector shapes
pU = pU(:);
p_user = p_user(:);

% BS is at origin
p_BS = [0; 0; 0];

% Transform BS and user positions to UAV local frame
p_BS_local = R' * (p_BS - pU);
p_user_local = R' * (p_user - pU);

% Elevation cosine angles (z in local frame points out)
cos_phi1 = max(0, min(1, -p_BS_local(3) / norm(p_BS - pU)));
cos_phi2 = max(0, min(1, -p_user_local(3) / norm(p_user - pU)));

% Effective aperture gain
F = cos_phi1 * cos_phi2;

end
