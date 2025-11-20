function pathXY_smooth = smoothPath(pathXY, w)
% smoothPath Smooths a 2D path using moving average
%   pathXY: Nx2 matrix of 2D coordinates [x y]
%   w: window size for moving average
%   Returns:
%       pathXY_smooth: Nx2 matrix of smoothed coordinates

    % Input validation (optional)
    if size(pathXY, 2) ~= 2
        error('Input pathXY must be an Nx2 matrix.');
    end

    % Smooth x and y components separately
    x_smooth = smoothdata(pathXY(:,1), 'movmean', w);
    y_smooth = smoothdata(pathXY(:,2), 'movmean', w);

    % Combine into a single matrix
    pathXY_smooth = [x_smooth, y_smooth];
end
