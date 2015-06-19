%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compares answers from shiftFunnel_qcqp_forces_mex and replanFunnels_mex 
% on a funnel library with a single funnel to make sure that the shifted 
% funnel positions they both give are the same.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
addpath('../');
addpath('../Visualization/');
addpath('../resolveCollision/');

%%%%% Set test options%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
numExperiments = 1000; % Number of random environments to test on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load funnel library
% load('../../FunnelLibrary/funnelLibraryArtificial.mat');
load('../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');

for exp = 1:numExperiments
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Random polytope forest
    numObs = randi(10);
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
    
    % First call QCQP shifting
    [~, x_opt_mex] = shiftFunnel_qcqp_forces_mex(x_current,forest,funnelLibrary,funnelIdx); 
    
    % Then call replan funnels
    options = struct();
    options.shift_method = 'qcqp';
    options.penetration_thresh = -10.0;
    options.failsafe_penetration = -10.0;
    [nextFunnel,x_execute_next] = replanFunnels_mex(x_current,forest,funnel,options);
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Perfom checks here
    
    % Check that we get same answer from planner and snopt shifting
    if any(abs(x_opt_mex - x_execute_next) > 1e-3)
        error('replanFunnels and shiftFunnel_qcqp_forces_mex gave different answers.');
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
end


disp('Tests passed!');


