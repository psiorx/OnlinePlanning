close all;
addpath('./Visualization/');
addpath('./OnlinePlanner/resolveCollision');
addpath('./OnlinePlanner/');

% Load hoop
load('./PoissonForest/hoopObs.mat');
forest = hoop;

% Load funnel library
load('./FunnelLibrary/funnelLibrary_Mar28.mat');

% Specify current position of airplane and funnel number
funnelIdx = 1;
funnel = funnelLibrary(funnelIdx);
x0 = funnel.x0(:,1);
x_current = x0;
xyz = funnel.xyz;
xyz_shift = xyz(:,end-2); % xyz_shift(2) = xyz_shift(2) + 0.20;
x_current(1:3) = -xyz_shift;
x_current(4:end) = x_current(4:end); % + 0.02*randn(9,1);
x_guess = x_current(1:3);


%%%% Call replan funnels with qcqp shifting %%%%%%%%%%%%%%%%%%%%%%%%%%%
options = struct();
options.shift_method = 'qcqp';
options.penetration_thresh = -10.0;
options.failsafe_penetration = -10.0;
tic
[nextFunnel,x_execute_next,collFree,min_dist] = replanFunnels_mex(x_current,forest,funnelLibrary,options);
toc
min_dist


% Plot stuff
figure
hold on
for k = 1:length(forest)
    P = polytope(forest{k}');
    plot(P);
end

% Plot funnel
V = funnel.V;
plotopts.num_samples = 1000;
plotopts.ts = funnel.ts;
plotopts.x0 = funnel.xtraj;
plotopts.inclusion = 'projection';
plotopts.inflate_radius = 0.22;

plotShiftedFunnel(V,[x_execute_next;zeros(9,1)],plotopts);
hold on

% plotopts.inflate_radius = 0.0;
% plotShiftedFunnel(V,[x_execute_next;zeros(9,1)],plotopts);

% REVERSE DIRECTION OF PLOTTING (to match coordinate frame)
axis([-5 1 -3 3 -2 2]);
set(gca,'YDir','reverse')
set(gca,'ZDir','reverse')

