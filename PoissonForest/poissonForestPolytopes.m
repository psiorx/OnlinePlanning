function [forest, forestMPT] = poissonForestPolytopes(xlims,ylims,lambda,tree_height)

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

% Create polytopes around these points
forest = cell(1,N);
forestMPT = [];
theta_range = 10*pi/180; % radians
dist_range = 0.1;

% tree_height = 2; % m
for k = 1:N
    % Generate polytope with random sides
    theta1 = 135*pi/180 + (2*rand(1)-1)*theta_range;
    n1 = [cos(theta1) sin(theta1) 0];
    
    theta2 = 225*pi/180 + (2*rand(1)-1)*theta_range;
    n2 = [cos(theta2) sin(theta2) 0];
    
    theta3 = 0*pi/180 +  (2*rand(1)-1)*theta_range;
    n3 = [cos(theta3) sin(theta3) 0];
    
    % Add last two  in z
    Aineq = [n1;n2;n3;0 0 -1;0 0 1];
    bineq = [dist_range*ones(3,1);tree_height/2;tree_height/2];
   
    Treek = polytope(Aineq, bineq) + [xy(:,k);0];
    forestMPT = [forestMPT, Treek];
    
    [H,K] = double(Treek);
    
    %forest{k}.Aineq = H(1:3,1:3);
    %forest{k}.bineq = K(1:3);
    forest{k} = extreme(Treek)';
         
end


end

