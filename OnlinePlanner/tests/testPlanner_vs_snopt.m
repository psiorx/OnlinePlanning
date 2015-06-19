%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compares answers from shiftFunnel_snopt_mex and replanFunnels_mex on a
% funnel library with a single funnel to make sure that the shifted funnel
% positions they both give are the same. replanFunnels_mex uses snopt
% inside it, so they should always be the same.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
addpath('../');
addpath('../Visualization/');
addpath('../resolveCollision_snopt/');

%%%%% Set test options%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
numExperiments = 100; % Number of random environments to test on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load funnel library
% load('../../FunnelLibrary/funnelLibraryArtificial.mat');
load('../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');

for exp = 1:numExperiments
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Random polytope forest
    numObs = randi(2);
    % Generate random obstacles
    for k = 1:numObs
        numVerts = randi(10) + 5;
        forest{k} = randn(3,numVerts);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Specify current position of airplane and funnel number
    funnelIdx = 1;
    funnel = funnelLibrary(funnelIdx);
    x0 = funnel.x0(:,1);
    x_current = x0;
    x_current(1:3) = x_current(1:3) + mean(forest{1},2); % Makes sure it's in collision. Otherwise x_execute_next will just be x_current.
    x_current(4:end) = x_current(4:end) + 0.02*randn(9,1);
    x_guess = x_current(1:3);
    
    % First call snopt shifting
    [~,x_opt_mex] = shiftFunnel_snopt_mex(x_current,forest,funnel,1);
    
    % Then call replan funnels
    options = struct();
    options.penetration_thresh = -10.0;
    options.failsafe_penetration = -10.0;
    options.shift_method = 'snopt';
    [nextFunnel,x_execute_next] = replanFunnels_mex(x_current,forest,funnel,options);
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Perfom checks here
    
    %if info == 1
        % Check that we get same answer from planner and snopt shifting
        if any(abs(x_opt_mex - x_execute_next) > 1e-3)
            error('replanFunnels and shiftFunnel_snopt_mex gave different answers.');
        end
    %end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
end


disp('Tests passed!');


return;



% Polytope
% forest{1} = 0.5*randn(3,5);
% rotMat = [cos(pi/4), -sin(pi/4), 0;sin(pi/4) cos(pi/4) 0; 0 0 1];
for k = 1:15
forest{k} = 10*[0 0 -2/5;0.1 0 -2/5;0.1 0.1 -2/5;0 0.1 -2/5;0 0 2/5;0.1 0 2/5;0.1 0.1 2/5;0 0.1 2/54]' + repmat([0.2+k*0.1;0.4;0],1,8);
end


for k = 1:length(forest)
    hold on
    K = convhull(forest{k}(1,:),forest{k}(2,:),forest{k}(3,:));
    % clf
    trisurf(K,forest{k}(1,:),forest{k}(2,:),forest{k}(3,:),'FaceColor','g');
    alpha(0.9);
end

funnel = funnelLibrary(1); % return;

x0 = funnel.x0(:,1); 
x0(1:3) = [0;0;0];

for k = 1:1
% Call planner
tic
[nextFunnel,x_execute_next] = replanFunnels_mex(x0,forest,funnel);
toc
end
