%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compares answers from replanFunnels_mex to isCollisionFree_mex.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
addpath('../');
addpath('../Visualization/');
addpath('../resolveCollision');

%%%%% Set test options%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
numExperiments = 100; % Number of random environments to test on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load funnel library
% load('../../FunnelLibrary/funnelLibraryArtificial.mat');
load('../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');
% load('../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');

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
    x_current(1:3) = x_current(1:3) + mean(forest{1},2) + 1*randn(3,1);
    x_current(4:end) = x_current(4:end) + 0.02*randn(9,1);
    x_guess = x_current(1:3);
    
    
    %%%% Call replan funnels with qcqp shifting %%%%%%%%%%%%%%%%%%%%%%%%%%%
    options = struct();
    options.shift_method = 'qcqp';
    options.penetration_thresh = -10.0;
    options.failsafe_penetration = -10.0;
    [nextFunnel,x_execute_next,collFree_planner,min_dist_planner] = replanFunnels_mex(x_current,forest,funnelLibrary,options); 
    
    % Check that we got the correct penetration depth and collisionFree
    % flag
    x_shifted_full = [x_execute_next;x0(4:end)];
    [collFree,min_dist] = isCollisionFree_mex(x_shifted_full,forest,funnelLibrary,nextFunnel);
    
    if collFree ~= collFree_planner
        error('Collision free flag from planner is not correct.');
    end
    
    if abs(min_dist - min_dist_planner) > 1e-3
        warning('Penetration distance from planner is not correct. This is possible but rare.');
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%% Call replan funnels with snopt shifting %%%%%%%%%%%%%%%%%%%%%%%%%%%
    options = struct();
    options.shift_method = 'snopt';
    options.penetration_thresh = -10.0;
    options.failsafe_penetration = -10.0;
    [nextFunnel,x_execute_next,collFree_planner,min_dist_planner] = replanFunnels_mex(x_current,forest,funnelLibrary,options);
    
    % Check that we got the correct penetration depth and collisionFree
    % flag
    x_shifted_full = [x_execute_next;x0(4:end)];
    [collFree,min_dist] = isCollisionFree_mex(x_shifted_full,forest,funnelLibrary,nextFunnel);
    
    if collFree ~= collFree_planner
        error('Collision free flag from planner is not correct');
    end
    
    if abs(min_dist - min_dist_planner) > 1e-3
        error('Penetration distance from planner is not correct');
    end
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
      %%%% Call replan funnels with no shifting %%%%%%%%%%%%%%%%%%%%%%%%%%%
    options = struct();
    % options.shift_method = 'snopt';
    options.penetration_thresh = -10.0;
    options.failsafe_penetration = -10.0;
    [nextFunnel,x_execute_next,collFree_planner,min_dist_planner] = replanFunnels_mex(x_current,forest,funnelLibrary,options);
    
    % Check that we got the correct penetration depth and collisionFree
    % flag
    x_shifted_full = [x_execute_next;x0(4:end)];
    [collFree,min_dist] = isCollisionFree_mex(x_shifted_full,forest,funnelLibrary,nextFunnel);
    
    if collFree ~= collFree_planner
        error('Collision free flag from planner is not correct');
    end
    
    if abs(min_dist - min_dist_planner) > 1e-3
        error('Penetration distance from planner is not correct');
    end
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load funnel library

clear forest funnelLibrary;
load('../../FunnelLibrary/funnelLibrary_Apr7.mat');

% Length of library
N = length(funnelLibrary);

numExps = 100;

for exp = 1:numExps
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Forest
    forest = {};
    
    % For each funnel, place an obstacle on it with some probability p
    p = 0.5;
    for k = 1:N
        if  (rand(1) < p)
            forest{end+1} = 0.1*randn(3,6);
            xyz_end = funnelLibrary(k).xyz(:,end-1);
            forest{end} = forest{end} + repmat(xyz_end,1,6);
        end
    end
    
    if isempty(forest)
        forest{1} = 0.1*randn(3,6);
        xyz_end = funnelLibrary(1).xyz(:,end-1);
        forest{1} = forest{1} + repmat(xyz_end,1,6);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    x_current = funnelLibrary(1).x0(:,1);
    
    %%%% Call replan funnels with qcqp shifting %%%%%%%%%%%%%%%%%%%%%%%%%%%
    options = struct();
    options.shift_method = 'qcqp';
    options.penetration_thresh = -100.0;
    options.failsafe_penetration = -100.0;
    [nextFunnel,x_execute_next,collFree_planner,min_dist_planner] = replanFunnels_mex(x_current,forest,funnelLibrary,options); 
    
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Now check that each funnel before nextFunnel was in collision
   if nextFunnel ~= 0      
     ind_check = 1:nextFunnel-1;
   else
       ind_check = 1:nextFunnel;
   end
   
   for k = ind_check
       [collFree_forces, x_opt_forces] = shiftFunnel_qcqp_forces_mex(x_current,forest,funnelLibrary,k);
       % [x_opt_forces,collFree_forces,tau_forces,exitflag_forces,info] = shiftFunnel_qcqp_forces(x_current,forest,funnelLibrary,k);
       
       if collFree_forces
           error('There was a funnel before nextFunnel that was collision free. Yet, the planner did not return this funnel');
       end
   end   
   
   % Now do planning with just nextFunnel and make sure answers are same
   [~,x_execute_next2,collFree_planner2,min_dist_planner2] = replanFunnels_mex(x_current,forest,funnelLibrary(nextFunnel),options);
   
   if any(abs(x_execute_next - x_execute_next2) > 1e-3)
       error('Planning with nextFunnel gave a different x_execute_next');
   end
   
   if abs(min_dist_planner - min_dist_planner2) > 1e-3
       error('Planning with nextFunnel gave a different min_dist');
   end
   
   if collFree_planner ~= collFree_planner2
       error('Planning with nextFunnel gave a different collFree');
   end
   
   
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






disp('Tests passed!');
























