function [pathXYPU] = optimalTrajectoryPU( ...
                           PUvec, x, y, p_init, p_final, varargin)


% ---- optional argument -------------------------------------------------
p = inputParser;
addParameter(p,'k',6,@(v) isnumeric(v)&&v>=2);
parse(p,varargin{:});        k = round(p.Results.k);

N = numel(PUvec);
X = [x(:), y(:)];                       % N×2 coordinates

% ---- nearest nodes to start/end ---------------------------------------
[~,startNode] = min( sum((X - p_init ).^2, 2) );
[~,endNode  ] = min( sum((X - p_final).^2, 2) );

% ---- build k-NN graph --------------------------------------------------
idx = knnsearch(X, X, 'K', k+1);        % self + k neighbours


edges = [];                             % [src dst w] rows
for n = 1:N
    neigh = idx(n,2:end);               % drop self
    nn    = numel(neigh);
    src   = repmat(n ,nn,1);            % column
    dst   = neigh(:);                   % column
    w     = (PUvec(dst)).';          % column  <-- fixed
    edges = [edges; [src dst w]];
end

G = digraph(edges(:,1), edges(:,2), edges(:,3), N);

% ---- shortest path (min cost ⇒ max rate) ------------------------------
[idxPath,~] = shortestpath(G,startNode,endNode,'Method','positive');

pathXYPU    = X(idxPath, :);        

end