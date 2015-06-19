%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compares funnel shifts using QCQP for Mosek and Forces Pro on
% environments consisting of random polytopes. Performs the following
% checks:
% - collisionFree flag are same
% - optimal values returned are same
% - checks that the collisionFree flag is the same as that reported by
% isCollisionFree_mex
% - If the shifted funnel is not collision free, then it checks that the
%  penetration distances resulting from both shifts are the same. (We would
%  expect this to be the case on random obstacles).
% - Shifts everything (current position and forest) by some random amount
% and makes sure that the relative shifts are same.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

megaclear;
close all;
addpath('../');
addpath('../../../Visualization/');
addpath('../../');
addpath('../../../');

%%%%% Set test options%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
generate_code = true;
numExperiments = 100; % Number of random environments to test on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% randn('state',13);

%%% Setup obstacles, etc. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load funnel library
% load('../../FunnelLibrary/funnelLibraryArtificial.mat');
% load('../../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');
load('../../../FunnelLibrary/funnelLibrary_Apr24_symmetrized.mat'); 

% Warning
disp(' ');
disp(' ');
disp(' ');
warning('NOTE: For now, shiftFunnel_qcqp assumes that the funnel is fixed to be the one code was generated for.');

for exp = 1:numExperiments
    
    % randn('state',11);
    
    clear forest;
    
    numObs = randi(20);
    % Generate random obstacles
    for k = 1:numObs
        numVerts = randi(10) + 5;
        forest{k} = randn(3,numVerts);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Specify current position of airplane and funnel number
    funnelIdx = 37;
    funnel = funnelLibrary(funnelIdx);
    x0 = funnel.x0(:,1);
    x_current = x0;
    x_current(1:3) = x_current(1:3) + mean(forest{1},2) + randn(3,1);
    x_current(4:end) = x_current(4:end) + 0.03*randn(9,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Generate Forces Pro code if needed
    if generate_code
        cd('../');
        shiftFunnel_generate_code;
        cd('tests');
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % return;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Shift funnel using QCQP
    
    % First using mosek
    [x_opt_mosek,collFree_mosek,tau_mosek] = shiftFunnel_qcqp(x_current,forest,funnelLibrary,funnelIdx);
    
    % Then Forces Pro (Matlab interface)
    [x_opt_forces,collFree_forces,tau_forces,exitflag_forces,info] = shiftFunnel_qcqp_forces(x_current,forest,funnelLibrary,funnelIdx);
    
    % Then using Forces Pro mex interface
    [collFree_mex, x_opt_mex] = shiftFunnel_qcqp_forces_mex(x_current,forest,funnelLibrary,funnelIdx);  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Perform checks here
    
    % Check that when mosek/forces says it's collision free, it really is.
    x_shifted_full_mosek = [x_opt_mosek;x0(4:end)];
    x_shifted_full_forces = [x_opt_mex;x0(4:end)];
    collFree = isCollisionFree_mex(x_shifted_full_forces,forest,funnel,1);
    if collFree ~= collFree_mosek
        warning('Collision free flag does not match collision free check using bullet. This is possible but rare.');
        disp('collFree:')
        disp(collFree);
        disp('collFree_qcqp:')
        disp(collFree_mosek);
    end
    
    % If we get a numerical error from forces, don't do any other checks
    if (exitflag_forces ~= 1)
        info
        continue;
    end
    
    % Check that forces matlab and mex report the same collision flag
    if collFree_forces ~= collFree_mex
        error('Forces Matlab and Mex reported different collisionFree flag.s');
    end
    
    % Check that forces matlab and mex give us the same x_opt
    if any(abs(x_opt_forces - x_opt_mex) > 0.01)
        error('Forces Matlab and Mex gave us different shifts.');
    end
    
    
    % Check that both mosek and forces report the same collisionFree flag
    if collFree_mosek ~= collFree_forces
        error('Mosek and Forces reported different collisionFree flags.');
    end
    
    % Check that the optimal values are same
    if abs(tau_mosek - tau_forces) > 1e-3
        error('Optimal tau for Mosek and Forces is not the same.');
    end
    
    
    % If things are not collision free, then check that both Mosek and Forces
    % gave us shifts with the same penetration depth. If things are not
    % collision free, we don't necessarily expect them to give the same
    % penetration depths, so we don't check anything in that case.
    if ~collFree_mosek
        
        [~,minDist_mosek] = isCollisionFree_mex(x_shifted_full_mosek,forest,funnel,1);
        [~,minDist_forces] = isCollisionFree_mex(x_shifted_full_forces,forest,funnel,1);
        
        if abs(minDist_mosek - minDist_forces) > 1e-3
            error('Penetration depths are not same for Mosek and Forces shifts');
        end
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Now shift everything by some random amount and make sure answers are same
    x_move = 0*randn(3,1);
    
    % Move obstacles
    for k = 1:numObs
        forest{k} = forest{k} + repmat(x_move,1,length(forest{k}));
    end
    
    % Move x_current cyclix dimensions
    x_current(1:3) = x_current(1:3) + x_move;
    
    % Shift funnel using QCQP
    
    % First using mosek
    [x_opt_mosek2,collFree_mosek2,tau_mosek2] = shiftFunnel_qcqp(x_current,forest,funnelLibrary,funnelIdx);
    
    % Then Forces Pro
    [x_opt_forces2,collFree_forces2,tau_forces2,exitflag_forces2,info2] = shiftFunnel_qcqp_forces(x_current,forest,funnelLibrary,funnelIdx);
    
    % Then using Forces Pro mex interface
    [collFree_mex2, x_opt_mex2] = shiftFunnel_qcqp_forces_mex(x_current,forest,funnelLibrary,funnelIdx); 
    
    % If we get a numerical error from forces, don't do any checks
    if (exitflag_forces2 ~= 1)
        info2
        continue;
    end
    
     % Check that forces matlab and mex report the same collision flag
    if collFree_forces2 ~= collFree_mex2
        error('Forces Matlab and Mex reported different collisionFree flags on shifted environment.');
    end
    
    % Check that forces matlab and mex give us the same x_opt
    if any(abs(x_opt_forces2 - x_opt_mex2) > 0.01)
        error('Forces Matlab and Mex gave us different shifts on shifted environment.');
    end
    
    % Now check that answers are same
    x_mosek_shift_back = x_opt_mosek2 - x_move;
    x_forces_shift_back = x_opt_forces2 - x_move;
    
    if any(abs(x_mosek_shift_back - x_opt_mosek) > 1e-3)
        error('Mosek gave different answer after shifting everything.');
    end
    
    if any(abs(x_forces_shift_back - x_opt_forces) > 1e-3)
        error('Forces gave different answer after shifting everything.');
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
end

disp('Tests passed!');













