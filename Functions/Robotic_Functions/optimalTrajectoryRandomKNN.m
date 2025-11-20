%% ---------- FUNCTION: k-NN optimal-rate path --------------------------
function [pathXY,idxPath] = optimalTrajectoryRandomKNN( ...
                           Rvec, x, y, p_init, p_final, varargin)
% OPTIMALTRAJECTORYRANDOMKNN   Maximum-rate path through an unstructured
%                              random node cloud using a k-nearest-neighbour
%                              graph and Dijkstra search.
%
%   [pathXY,totalRate] = ...('k',8)   sets k = 8 neighbours per node.
% -----------------------------------------------------------------------

% ---- optional argument -------------------------------------------------
p = inputParser;
addParameter(p,'k',6,@(v) isnumeric(v)&&v>=2);
parse(p,varargin{:});        k = round(p.Results.k);

N = numel(Rvec);
X = [x(:), y(:)];                       % N×2 coordinates

% ---- nearest nodes to start/end ---------------------------------------
[~,startNode] = min( sum((X - p_init ).^2, 2) );
[~,endNode  ] = min( sum((X - p_final).^2, 2) );

% ---- build k-NN graph --------------------------------------------------
idx = knnsearch(X, X, 'K', k+1);        % self + k neighbours
C   = max(Rvec) + eps;                  % positive offset

edges = [];                             % [src dst w] rows
for n = 1:N
    neigh = idx(n,2:end);               % drop self
    nn    = numel(neigh);
    src   = repmat(n ,nn,1);            % column
    dst   = neigh(:);                   % column
    w     = (C - Rvec(dst)).';          % column  <-- fixed
    edges = [edges; [src dst w]];
end

G = digraph(edges(:,1), edges(:,2), edges(:,3), N);

% ---- shortest path (min cost ⇒ max rate) ------------------------------
[idxPath,totalCost] = shortestpath(G,startNode,endNode,'Method','positive');

pathXY    = X(idxPath, :);
totalRate = (numel(idxPath)-1)*C - totalCost;
end

