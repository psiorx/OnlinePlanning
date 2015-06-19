function forest = poissonForestCylinders(xlims,ylims,rlims,lambda)

% % Limits of forest
% xlims = [0 10];
% ylims = [0 50];
% 
% % Forest density
% lambda = 0.3; % trees per m^2

% Compute area
area = diff(xlims)*diff(ylims);

% Generate number of trees from Poisson distribution
N = poissrnd(lambda*area)

% Generate position of trees
xy = rand(2,N);
xy(1,:) = (xy(1,:)*(xlims(2) - xlims(1))) + xlims(1);
xy(2,:) = (xy(2,:)*(ylims(2) - ylims(1))) + ylims(1);

% Create cylinders around these points
forest = zeros(3,N);
forest(1:2,:) = xy;
rs = randn(1,N);
forest(3,:) = (rs*(rlims(2) - rlims(1))) + rlims(1);

forest = forest';



end

