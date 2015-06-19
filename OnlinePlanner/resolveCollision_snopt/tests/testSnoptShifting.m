%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Checks that the matlab and mex implementations of shifting funnels using
% snopt give same answers. We do this check on random polytopic
% environments. Also checks that if we shift everything (current state and
% forest) we get the same results.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

megaclear;
clear all;
close all;
addpath('../');
addpath('../../../Visualization/');
addpath('../../resolveCollision/');


%%%%% Set test options%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
numExperiments = 100; % Number of random environments to test on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load funnel library
% load('../../FunnelLibrary/funnelLibraryArtificial.mat');
load('../../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');

for exp = 1:numExperiments
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clear forest;
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
    x_current(1:3) = x_current(1:3) + mean(forest{1},2) + randn(3,1);
    x_current(4:end) = x_current(4:end) + 0.02*randn(9,1);
    x_guess = zeros(3,1);
    
    userfun = @(x) snoptWrapperFun(x,funnel,forest,x_current); %  [0;collisionConstraint(x,funnel,forest)];
    
    global SNOPT_USERFUN;
    SNOPT_USERFUN = userfun;
    
    % snset('Major feasibility tolerance=1e-3'); %
    % snset('Major optimality tolerance=1e-3'); %
    % % snset('Minor feasibility tolerance=1e-3'); %
    % % snset('Minor optimality tolerance=1e-3'); %
    % snset('Derivative Option=1');
    % snset('Verify Level=0');
    % snset('Major Iterations Limit=100');
    
    % Setup call to snopt
    xlow = -Inf*ones(3,1);
    xupp = Inf*ones(3,1);
    
    Flow = [-Inf;-Inf];
    Fupp = [Inf;1];
    
    disp(' ');
    
    disp('Matlab:');
    [x_opt_matlab,F,info,xmul,Fmul] = snopt(x_guess,xlow,xupp,Flow,Fupp,'snoptDummyFun',0,1,[],[],[],[1;2;1;2;1;2],[1;1;2;2;3;3]);
    x_opt_matlab = x_opt_matlab + x_current(1:3); % Add in x_current to get SHIFTED funnel
    disp(['Info:' num2str(info)])
    
    disp('MEX:');
    [collFree_snopt,x_opt_mex,min_dist_snopt] = shiftFunnel_snopt_mex(x_current,forest,funnelLibrary,funnelIdx); % This already returns SHIFTED position
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Perfom checks here
    
    % Check that we got the correct penetration depth from snopt
    x_shifted_full = [x_opt_mex;x0(4:end)];
    [collFree,min_dist] = isCollisionFree_mex(x_shifted_full,forest,funnel,1);
    
    if collFree ~= collFree_snopt
        error('Collision free flag from snopt is not correct');
    end
    
    if abs(min_dist - min_dist_snopt) > 1e-3
        error('Penetration distance from snopt is not correct');
    end
    
    if info == 1
        % Check that we get same answer from matlab and mex
        if any(abs(x_opt_matlab - x_opt_mex) > 1e-3)
            error('Matlab and mex gave different shifts.');
        end
        
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Shift everything by some amount and check that we get same answer
    x_move = 10*randn(3,1);
    
    clear forest_shifted
    % Move obstacles
    for k = 1:numObs
        forest_shifted{k} = forest{k} + repmat(x_move,1,length(forest{k}));
    end
    
    % Move x_current cyclix dimensions
    x_current(1:3) = x_current(1:3) + x_move;
    x_guess = zeros(3,1);
    
    % Shift with matlab first
    userfun = @(x) snoptWrapperFun(x,funnel,forest_shifted,x_current); %  [0;collisionConstraint(x,funnel,forest)];
    
    SNOPT_USERFUN = userfun;
    
    % Call matlab snopt
    [x_opt_matlab2,F2,info2] = snopt(x_guess,xlow,xupp,Flow,Fupp,'snoptDummyFun',0,1,[],[],[],[1;2;1;2;1;2],[1;1;2;2;3;3]);
    x_opt_matlab2 = x_opt_matlab2 + x_current(1:3);
    
    % Now mex
    [~,x_opt_mex2] = shiftFunnel_snopt_mex(x_current,forest_shifted,funnelLibrary,funnelIdx); % This already returns SHIFTED position
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Perform checks here
    x_matlab_shift_back = x_opt_matlab2 - x_move;
    x_mex_shift_back = x_opt_mex2 - x_move;
    
    %     if (info ~= info2)
    %         error('Snopt info before and after shifting are different');
    %     end
    
    if (info == 1) && (info2 == 1)
        if any(abs(x_matlab_shift_back - x_opt_matlab) > 1e-3)
            error('Matlab gave different answer after shifting everything.');
        end
        
        if any(abs(x_mex_shift_back - x_opt_mex) > 1e-3)
            error('Mex gave different answer after shifting everything.');
        end
        
    end
    
    
    
end


disp('Tests passed!');