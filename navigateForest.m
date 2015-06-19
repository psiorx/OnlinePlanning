megaclear;
close all 
% Add folders to path
% addpath(pwd);
addpath('OnlinePlanner');
addpath('PoissonForest');
addpath('Visualization/');
addpath('Util');
% addpath('/home/anirudha/Software/pods/drake/systems/plants/collision/test');

% Generate Poisson Forest

% Limits of forest
xlims = [1 10]; % [1 5];
ylims = [-3 3];
rlims = [0.1 0.2];
% Forest density
lambda = 0.2; % 0.1 trees per m^2
% Height of tree
tree_height = 2;

load goodForests4.mat
% [forest, forestMPT] = poissonForestPolytopes(xlims,ylims,lambda,tree_height); % return; 

% forest = poissonForestCylinders(xlims,ylims,rlims,lambda);

% Load funnel library
% load ./FunnelLibrary/funnelLibraryArtificial.mat
load ./FunnelLibrary/funnelLibrary6.mat
% load ../../Quadrotor/FunnelLibrary/funnelLibrary.mat


% Specify initial condition
x0 = funnelLibrary(1).x0(:,1);
% x0 = zeros(12,1); x0(1) = 0; x0(2) = 0; x0(7) = 6;


% Create online planner object
op = OnlinePlannerMex(funnelLibrary,forest,1,[1;2;3],0.75,0,x0,4); % t_replan = 1.5
% return;


%tic;
%nextFunnel = op.replanFunnels(0, x0);
%toc


% tic; 
% collFree = op.isCollisionFree(1,x0);
% toc

% Cascade with plane
% p = PlanePlantAni();
% p = PlanePlantIntegrator();

op = op.setInputFrame(p.getStateFrame);
op = op.setOutputFrame(p.getInputFrame);

% op = op.setInputFrame(funnelLibrary(1).controller.getInputFrame);
% op = op.setOutputFrame(funnelLibrary(1).controller.getOutputFrame);

sysCl = feedback(p,op);

% Navigate through forest
%mdl = getModel(sysCl);
%set_param(mdl,'Solver','ode1'); % Set solver to euler to avoid difficulties with variable steps and replanning of funnels
% return;
%xtrajSim = sysCl.simulate([0 1.1],x0);
xtrajSim = simulateEuler(sysCl,0.8,x0,0.001);

% figure
% plot(forestMPT,'g');
% hold on
% fnplt(xtrajSim);
% axis equal

return;


v = SBachVisualizer(p,forestMPT,xlims,ylims,tree_height);
v.playback_speed = 1.0;
% v.playback(xtrajSim);
v.playbackAVI(xtrajSim,'forest4.avi');


hold on;
ts = xtrajSim.getBreaks();
xss = xtrajSim.eval(ts);

plot3(xss(1,:),xss(2,:),xss(3,:),'LineWidth',3);

% fnplt(xtrajSim,[1 2 3]);

return;

funnelLibrary(1).xtraj = funnelLibrary(1).xtraj.setOutputFrame(p.getStateFrame);
funnelLibrary(2).xtraj = funnelLibrary(2).xtraj.setOutputFrame(p.getStateFrame);
funnelLibrary(3).xtraj = funnelLibrary(3).xtraj.setOutputFrame(p.getStateFrame);
funnelLibrary(4).xtraj = funnelLibrary(4).xtraj.setOutputFrame(p.getStateFrame);
funnelLibrary(5).xtraj = funnelLibrary(5).xtraj.setOutputFrame(p.getStateFrame);
funnelLibrary(6).xtraj = funnelLibrary(6).xtraj.setOutputFrame(p.getStateFrame);







return;


figure
hold on

% Plot funnel
k = 1;
options.ts = funnelLibrary(k).ts;
options.inclusion = 'projection';
options.x0 = funnelLibrary(k).xtraj;
% plotFunnel3(funnelLibrary(k).V,options);
plotShiftedFunnel(funnelLibrary(k).V,x0,options);

% Plot forest
plotopts.color = 'g';
plot(forestMPT,plotopts);


% REVERSE DIRECTION OF PLOTTING (to match coordinate frame)
set(gca,'YDir','reverse')
set(gca,'ZDir','reverse')


return;










 
 
 
 
 
 
 
