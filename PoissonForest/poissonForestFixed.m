function [forest, forestMPT] = poissonForestFixed(xobs,yobs,tree_height)

N = length(xobs);
% Generate position of trees
xy = zeros(2,N);
xy(1,:) = xobs;
xy(2,:) = yobs;

% Create polytopes around these points
forest = cell(1,N);
forestMPT = [];
theta_range = 10*pi/180; % radians
dist_range = 0.1; % 0.01

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

