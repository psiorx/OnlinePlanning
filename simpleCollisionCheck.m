% Add folders to path
addpath('OnlinePlanner');
addpath('PoissonForest');
addpath('Visualization/');
addpath('Util');

% Load funnel library
load('FunnelLibrary/funnelLibraryArtificial.mat'); 

% Generate Poisson Forest
% Limits of forest
xlims = [1 5];
ylims = [-3 3];
rlims = [0.1 0.2];
% Forest density
lambda = 0.1; % 0.15 trees per m^2
% Height of tree
tree_height = 2;

% load goodForests1.mat
[forest, forestMPT] = poissonForestPolytopes(xlims,ylims,lambda,tree_height); % return; 


x0 = zeros(12,1); 

% Do collision check
funnelIdx = 1; 
collisionFree = isCollisionFree_mex(x0,forest,funnelLibrary, funnelIdx)

% Plot forest
plot(forestMPT);

% Plot funnel
options.ts = funnelLibrary(funnelIdx).ts;
options.inclusion = 'projection';
options.x0 = funnelLibrary(funnelIdx).xtraj;
% plotFunnel3(funnelLibrary(k).V,options);
plotShiftedFunnel(funnelLibrary(funnelIdx).V,x0,options);