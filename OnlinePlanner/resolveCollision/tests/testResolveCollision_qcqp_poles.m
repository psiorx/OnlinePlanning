%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compares funnel shifts using QCQP for Mosek and Forces Pro on an
% environment consisting of poles. Checks that the optimal values are same
% and that the collisionFree flag returned by both are same.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

megaclear;
close all;
addpath('../');
addpath('../../../Visualization/');
addpath('../../');
addpath('../../../');

%%%%% Set test options%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
generate_code = false;
makePlots = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% randn('state',13);

%%% Setup obstacles, etc. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load funnel library
% load('../../FunnelLibrary/funnelLibraryArtificial.mat');
% load('../../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');
load('../../../FunnelLibrary/funnelLibrary_Apr7.mat');



% Generate poles
for k = 1:20
    forest{k} = 10*[0 0 -2/5;0.1 0 -2/5;0.1 0.1 -2/5;0 0.1 -2/5;0 0 2/5;0.1 0 2/5;0.1 0.1 2/5;0 0.1 2/54]' + repmat([0.2+k*0.1;0.2;0.0],1,8);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Specify current position of airplane and funnel number
funnelIdx = 1;
funnel = funnelLibrary(funnelIdx);
x0 = funnel.x0(:,1);
x_current = x0;
x_current(1:3) = x_current(1:3) + randn(3,1);
x_current(4:end) = x_current(4:end) + 0.02*randn(9,1);
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
% Shift funnel using QCQP

% First using mosek
[x_opt_mosek,collFree_mosek,tau_mosek] = shiftFunnel_qcqp(x_current,forest,funnelLibrary,funnelIdx);

% Then Forces Pro
warning('NOTE: For now, shiftFunnel_qcqp assumes that the funnel is fixed to be the one code was generated for.');
[x_opt_forces,collFree_forces,tau_forces] = shiftFunnel_qcqp_forces(x_current,forest,funnelLibrary,funnelIdx);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Perform checks here

% First check that both mosek and forces report the same collisionFree flag
if collFree_mosek ~= collFree_forces
    error('Mosek and Forces reported different collisionFree flags');
end

% Check that the optimal values are same
if abs(tau_mosek - tau_forces) > 1e-4
    error('Optimal tau for Mosek and Forces is not the same.');
end

% % If things are not collision free, then check that both Mosek and Forces
% % gave us shifts with the same penetration depth. If things are not
% % collision free, we don't necessarily expect them to give the same
% % penetration depths, so we don't check anything in that case.
% if ~collFree_mosek
%     x_shifted_full_mosek = [x_opt_mosek;x0(4:end)];
%     x_shifted_full_forces = [x_opt_forces;x0(4:end)];
%     
%     [~,minDist_mosek] = isCollisionFree_mex(x_shifted_full_mosek,forest,funnel,1);
%     [~,minDist_forces] = isCollisionFree_mex(x_shifted_full_forces,forest,funnel,1);
%     
%     if abs(minDist_mosek - minDist_forces) > 1e-3
%         error('Penetration depths are not same for Mosek and Forces shifts');
%     end
% end

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




