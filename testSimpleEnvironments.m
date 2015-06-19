close all;
clear all;
addpath('./Visualization/');
addpath('./OnlinePlanner/resolveCollision');
addpath('./OnlinePlanner/');

% Funnel library
% load('../OnlinePlanning/FunnelLibrary/funnelLibrary_Apr7.mat');
load('../OnlinePlanning/FunnelLibrary/funnelLibrary_Apr24_symmetrized.mat');

% Hoop
%load('./PoissonForest/hoopObs.mat');

% Initial conditions
load('~/SBach/Logs/initConditions_all.mat');


% Positions of poles
pos = [3.0,3.0,1.0;-0.75,0.75,0.0;0.0,0.0,0.01]';
A = [eye(3);-eye(3)];
d = 0.025;

% Generate poles
for k = 1:size(pos,1)
    b = [pos(k,1)+d;pos(k,2)+d;0;d-pos(k,1);d-pos(k,2);2];
    P = polytope(A,b);
    forest{k} = extreme(P)';
end

%for j = 1:length(hoop)
%    hoop{j} = hoop{j} + repmat([3.3;-0.1;-1.3],1,size(hoop{j},2));
%    forest{end+1} = hoop{j};
%end

% Specify current position of airplane and funnel number
x_current = x0s_xhat(:,1);


%%%% Call replan funnels with qcqp shifting %%%%%%%%%%%%%%%%%%%%%%%%%%%
options = struct();
options.shift_method = 'qcqp';
options.penetration_thresh = 0.3; % -10.0
options.failsafe_penetration = -10.0;
for k = 1:10
tic
[nextFunnel,x_execute_next,collFree,min_dist] = replanFunnels_mex(x_current,forest,funnelLibrary,options);
toc
end
min_dist
nextFunnel


% Plot stuff
figure
hold on
for k = 1:length(forest)
    P = polytope(forest{k}');
    plot(P);
end

% Plot funnel
funnel = funnelLibrary(nextFunnel);
V = funnel.V;
plotopts.num_samples = 200;
plotopts.ts = funnel.ts;
plotopts.x0 = funnel.xtraj;
plotopts.inclusion = 'projection';
plotopts.inflate_radius = 0.22;

plotShiftedFunnel(V,[x_execute_next;zeros(9,1)],plotopts);
hold on

% plotopts.inflate_radius = 0.0;
% plotShiftedFunnel(V,[x_execute_next;zeros(9,1)],plotopts);

% REVERSE DIRECTION OF PLOTTING (to match coordinate frame)
axis([-1 4 -3 3 -3 0]);
set(gca,'YDir','reverse')
set(gca,'ZDir','reverse')

