function pathXY_dense = densifyToNs(pathXY, Ns)
% densifyToNs  –  resample a 2-D trajectory so that it contains exactly Ns
%                 equally spaced points (arc-length parameterisation).
%
% INPUTS
%   pathXY   L×2   original way-points  [x y]
%   Ns       1×1   desired number of samples  (integer ≥ L)
%
% OUTPUT
%   pathXY_dense   Ns×2   densified trajectory
%
% Example
%   Ns = 500;                               % choose any integer ≥ size(pathXY,1)
%   pathXY_dense = densifyToNs(pathXY, Ns);

% ---------- safety check ------------------------------------------------
Lorig = size(pathXY,1);
if Ns < Lorig
    error('Ns must be >= the original number of points (%d).', Lorig);
end

% ---------- cumulative arc-length --------------------------------------
x = pathXY(:,1);
y = pathXY(:,2);
s = [0; cumsum( hypot(diff(x), diff(y)) )];   % hypot = sqrt(dx^2 + dy^2)

% ---------- equally spaced samples along the arc -----------------------
s_dense = linspace(0, s(end), Ns).';          % Ns×1

% ---------- linear interpolation ---------------------------------------
x_dense = interp1(s, x, s_dense);
y_dense = interp1(s, y, s_dense);

pathXY_dense = [x_dense y_dense];
end
