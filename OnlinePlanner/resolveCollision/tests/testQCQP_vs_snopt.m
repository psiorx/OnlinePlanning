%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compares answers from shiftFunnel_snopt_mex and shiftFunnel_qcqp_forces
% to make sure that they are the same on a PARTICULAR FIXED environment. In
% general, we don't expect these to give the same answers, but in the
% particular environment I've chosen they should.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

megaclear;
close all;
addpath('../');
addpath('../../../Visualization/');
addpath('../../');
addpath('../../../');
addpath('../../resolveCollision_snopt/');

%%%%% Set test options%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
generate_code = false;
makePlots = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


randn('state',1);

%%% Setup obstacles, etc. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load funnel library
% load('../../FunnelLibrary/funnelLibraryArtificial.mat');
load('../../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');

% Generate pole
forest{1} = 10*[0 0 -2/5;0.1 0 -2/5;0.1 0.1 -2/5;0 0.1 -2/5;0 0 2/5;0.1 0 2/5;0.1 0.1 2/5;0 0.1 2/54]' + repmat([0.2+0.1;0.2;0.0],1,8);
forest{1} = forest{1} + repmat([2;5;-7],1,length(forest{1}));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Specify current position of airplane and funnel number
funnelIdx = 1;
funnel = funnelLibrary(funnelIdx);
x0 = funnel.x0(:,1);
x_current = x0;
x_current(1:3) = x_current(1:3) + [2;5;-7];
x_current(4:end) = x_current(4:end) + 0.1*randn(9,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate Forces Pro code if needed
if generate_code
    cd('../');
    shiftFunnel_generate_code;
    cd('tests');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Shift funnel 

% First using Forces Pro QCQP
warning('NOTE: For now, shiftFunnel_qcqp assumes that the funnel is fixed to be the one code was generated for.');
[x_opt_forces,collFree_forces,tau_forces] = shiftFunnel_qcqp_forces(x_current,forest,funnelLibrary,funnelIdx);

% Then using snopt
[~,x_opt_snopt] = shiftFunnel_snopt_mex(x_current,forest,funnelLibrary,funnelIdx);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Perform checks here
% Check that we get same answer from QCQP and snopt
if any(abs(x_opt_forces - x_opt_snopt) > 1e-3)
    error('QCQP and Snopt gave different shifts.');
end

disp('Tests passed!');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot stuff if asked for
if makePlots
    figure
    title('Unshifted funnel');
    % Plot obstacles
    for k = 1:length(forest)
        hold on
        K = convhull(forest{k}(1,:),forest{k}(2,:),forest{k}(3,:));
        % clf
        trisurf(K,forest{k}(1,:),forest{k}(2,:),forest{k}(3,:),'FaceColor','g');
    end
    alpha(0.9);
    
    % Plot stuff
    V = funnel.V;
    plotopts.num_samples = 1000;
    plotopts.ts = funnel.ts;
    plotopts.x0 = funnel.xtraj;
    plotopts.inclusion = 'projection';
    
    % res.sol.itr.xx = cS\res.sol.itr.xx;
    
    % plot original funnel
    plotShiftedFunnel(V,x_current,plotopts);
    hold on
    
    % Draw another figure with just the shifted one
    figure
    title('Shifted funnel');
    for k = 1:length(forest)
        hold on
        K = convhull(forest{k}(1,:),forest{k}(2,:),forest{k}(3,:));
        % clf
        trisurf(K,forest{k}(1,:),forest{k}(2,:),forest{k}(3,:),'FaceColor','g');
    end
    alpha(0.9);
    % Now, plot shifted funnel
    plotShiftedFunnel(V,[x_opt_forces;zeros(9,1)],plotopts);
    
end
