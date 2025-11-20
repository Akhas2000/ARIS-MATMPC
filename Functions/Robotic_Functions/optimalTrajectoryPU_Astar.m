function [pathXYPU] = optimalTrajectoryPU_Astar(PUvec, x, y, p_init, p_final, varargin)

% ---- parameters ----
p = inputParser;
addParameter(p,'k',6,@(v) isnumeric(v)&&v>=2);
parse(p,varargin{:}); k = round(p.Results.k);

N = numel(PUvec);
X = [x(:), y(:)];

% ---- nearest start & end nodes ----
[~,startNode] = min(sum((X - p_init ).^2, 2));
[~,endNode  ] = min(sum((X - p_final).^2, 2));

% ---- k-NN graph ----
idx = knnsearch(X, X, 'K', k+1);
edges = cell(N,1);
for n = 1:N
    neigh = idx(n,2:end);
    w = PUvec(neigh);       % same as Dijkstra
    edges{n} = [neigh(:), w(:)];
end

% ---- A* ----
open = [startNode, 0, norm(X(startNode,:)-X(endNode,:))]; % [node, g, f]
cameFrom = zeros(N,1);
gScore = inf(N,1); gScore(startNode)=0;

while ~isempty(open)
    [~, i] = min(open(:,3));   % node with lowest f = g + h
    current = open(i,1);
    open(i,:) = [];
    if current == endNode
        break;
    end
    for e = 1:size(edges{current},1)
        neighbor = edges{current}(e,1);
        cost = edges{current}(e,2);
        tentative_g = gScore(current) + cost;
        if tentative_g < gScore(neighbor)
            cameFrom(neighbor) = current;
            gScore(neighbor) = tentative_g;
            f = tentative_g + norm(X(neighbor,:) - X(endNode,:)); % heuristic
            open = [open; [neighbor, tentative_g, f]];
        end
    end
end

% ---- reconstruct path ----
path = endNode;
while cameFrom(path(1)) ~= 0
    path = [cameFrom(path(1)); path];
end
pathXYPU = X(path,:);

end
