%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compares gradients provided by collisionConstraintMex with numerical
% gradients using finite difference. Also checks that the penetration depth
% reported by collisionConstraintMex is the same as that reported by
% isCollisionFree_mex. Finally, shifts everything (forest and current state)
% and checks that we get the same answer. These tests are done on random 
% polytopic environments.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

megaclear;
addpath('../');
addpath('../../../Visualization/');
addpath('../../resolveCollision/');

%%%%% Set test options%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
numExperiments = 1000; % Number of random environments to test on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load funnel library
% load('../../FunnelLibrary/funnelLibraryArtificial.mat');
load('../../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');

for exp = 1:numExperiments
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clear forest;
    % Random polytope forest
    numObs = randi(5);
    % Generate random obstacles
    for k = 1:numObs
        numVerts = randi(10) + 5;
        forest{k} = randn(3,numVerts);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Setup funnel
    funnel = funnelLibrary(1);
    funnelIdx = 1;
    
    % Setup random x_shifted
    x_shifted = mean(forest{1},2) + 2*randn(3,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Compute "analytical" gradients from collision constraint code and also
    % using finite differences.
    
    % Nominal function value and gradient from collision checking code
    [~,f,df] = collisionConstraintMex(x_shifted,forest,funnelLibrary, funnelIdx);
    [~,f2] = isCollisionFree_mex(x_shifted,forest,funnel, funnelIdx);
    
    % Denominator in finite difference
    del = 1e-5;
    
    % Perturb in x direction
    [~,fx] = collisionConstraintMex([del;0;0]+x_shifted,forest,funnelLibrary, funnelIdx);
    
    % Perturb in y direction
    [~,fy] = collisionConstraintMex([0;del;0]+x_shifted,forest,funnelLibrary, funnelIdx);
    
    % Perturb in z direction
    [~,fz] = collisionConstraintMex([0;0;del]+x_shifted,forest,funnelLibrary, funnelIdx);
    
    df_fd = ([fx;fy;fz] - f)'/del;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Perform checks
    
    if abs(f - f2) > 1e-3
        error('Penetration values from collisionConstraintMex and isCollisionFree_mex did not match.');
    end
    
    % Check finite difference and collision constraint gradient are same
    if any(abs(df - df_fd) > 0.01)
        error('Finite difference and bullet gradients did not match.');
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Shift forest and x_shifted to make sure collision constraint is shift
    % invariant
    x_move = 10*randn(3,1);
    
    clear forest_shifted
    % Move obstacles
    for k = 1:numObs
        forest_shifted{k} = forest{k} + repmat(x_move,1,length(forest{k}));
    end
    
    % Move x_current cyclix dimensions
    x_shifted(1:3) = x_shifted(1:3) + x_move;
    
    % Call collision constraint again with stuff shifted
    [~,f_shifted,df_shifted] = collisionConstraintMex(x_shifted,forest_shifted,funnelLibrary, funnelIdx);
    
    if abs(f_shifted - f) > 1e-5
        error('We got different f for collision constraint after shifting');
    end
    
    if any(abs(df_shifted - df) > 1e-5)
        error('We got different df for collision constraint after shifting');
    end
        
        
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
end

disp('Tests passed!');










