addpath('../../Visualization/');
addpath('../../PoissonForest/');
close all;
% randn('state',0);
% Obstacle
% forest{1} = [0+0.05 0 -0.5;1+0.05 0 -0.5;0+0.05 1 -0.5;0+0.05 0 1-0.5]';
forest{1} = 2*randn(3,10);
% forest{1} = 20*[0 0 -2/20;0.1 0 -2/20;0.1 0.1 -2/20;0 0.1 -2/20;0 0 2/20;0.1 0 2/20;0.1 0.1 2/20;0 0.1 2/20]' + repmat([0.3;0;0],1,8);
K = convhull(forest{1}(1,:),forest{1}(2,:),forest{1}(3,:));
clf
trisurf(K,forest{1}(1,:),forest{1}(2,:),forest{1}(3,:));
alpha(0.5);

%[forest, forestMPT] = poissonForestFixed(0.5,-0.3,2);
%plot(forestMPT);


% Load funnel library
load('../../FunnelLibrary/funnelLibraryArtificial.mat');

funnel = funnelLibrary(1);
V = funnel.V;

N = length(funnel.ts);

tic
x0s = funnel.xyz;

ds = zeros(N,1);
ns = zeros(N,3);

for k = 1:N
    % Compute shift required
    forestk{1} = forest{1} - repmat(x0s(:,k),1,size(forest{1},2));
    cS = funnel.cS{k};
    forestk_tr{1} = cS*forestk{1};
    [dk_tr,nk_tr] = testResolveCollision(forestk_tr); % keyboard;
    
%     veck = cS\(abs(dk_tr)*nk_tr);
%     ns(k,:) = (veck/norm(veck))';
%     ds(k) = sign(dk_tr)*norm(veck);

    ns(k,:) = nk_tr'*cS;
    ds(k) = dk_tr;

end

% Now, setup and solve QP
q = eye(3);
c = zeros(1,3);
a = ns;
blc = -ds;
cmd = 'minimize echo(0)';


tic
res = mskqpopt(q,c,a,blc,[],[],[],[],cmd);
toc


% % Now gurobi
% model.Q = sparse(q);
% model.obj = c';
% model.A = sparse(a);
% model.rhs = blc;
% model.sense = '>';
% model.lb = -Inf*ones(length(c),1);
% params.OutputFlag = false;
% tic
% result = gurobi(model,params);
% toc


% Plot stuff
plotopts.num_samples = 1000;
plotopts.ts = funnel.ts;
plotopts.x0 = funnel.xtraj;
plotopts.inclusion = 'projection';

% res.sol.itr.xx = cS\res.sol.itr.xx;

% plot original funnel
plotShiftedFunnel(V,zeros(12,1),plotopts);
hold on
% Now, plot shifted funnel
% plotShiftedFunnel(V,[res.sol.itr.xx;zeros(9,1)],plotopts);


% Draw another figure with just the shifted one
figure
trisurf(K,forest{1}(1,:),forest{1}(2,:),forest{1}(3,:));
alpha(0.5);
% Now, plot shifted funnel
plotShiftedFunnel(V,[res.sol.itr.xx;zeros(9,1)],plotopts);

return;

addpath('../../Visualization/');
close all;
% randn('state',0);
% Obstacle
% forest{1} = [0+0.05 0 -0.5;1+0.05 0 -0.5;0+0.05 1 -0.5;0+0.05 0 1-0.5]';
forest{1} = 2*randn(3,10);
K = convhull(forest{1}(1,:),forest{1}(2,:),forest{1}(3,:));
clf
trisurf(K,forest{1}(1,:),forest{1}(2,:),forest{1}(3,:));
alpha(0.5);

% Ellipsoid
Sr = randn(3,3);
S = Sr*Sr';
V = QuadraticLyapunovFunction(CoordinateFrame('sphere_frame',3,'x'),S,zeros(3,1),0);

% Transform ellipsoid to make it sphere (and transform obstacle
% accordingly)
cS = chol(S);

% Centers of sphere
x0s = [0 0 0; 0.3 0 0; 0.6 0 0]' + randn(1);
N = size(x0s,2);

ds = zeros(N,1);
ns = zeros(N,3);

for k = 1:N
    % Compute shift required
    forestk{1} = forest{1} - repmat(x0s(:,k),1,size(forest{1},2));
    forestk_tr{1} = cS*forestk{1};
    [dk_tr,nk_tr] = testResolveCollision(forestk_tr);
    
%     veck = cS\(abs(dk_tr)*nk_tr);
%     ns(k,:) = (veck/norm(veck))';
%     ds(k) = sign(dk_tr)*norm(veck);

    ns(k,:) = nk_tr'*cS;
    ds(k) = dk_tr;

end

% Now, setup and solve QP
q = S; % eye(3);
c = zeros(1,3);
a = ns;
blc = -ds;
cmd = 'minimize echo(0)';

tic
res = mskqpopt(q,c,a,blc,[],[],[],[],cmd);
toc

% Plot stuff
plotopts.num_samples = 1000;

% res.sol.itr.xx = cS\res.sol.itr.xx;

% plot original funnel
for k = 1:N
    x_shifted = x0s(:,k); % + res.sol.itr.xx;
    hold on
    plotShiftedFunnel(V,x_shifted,plotopts);
    axis equal
end

% Now, plot shifted funnel
for k = 1:N
    x_shifted = x0s(:,k) + res.sol.itr.xx;
    hold on
    plotShiftedFunnel(V,x_shifted,plotopts);
    axis equal
end

% Draw another figure with just the shifted one
figure
trisurf(K,forest{1}(1,:),forest{1}(2,:),forest{1}(3,:));
alpha(0.5);

hold on
% Now, plot shifted funnel
for k = 1:N
    x_shifted = x0s(:,k) + res.sol.itr.xx;
    hold on
    plotShiftedFunnel(V,x_shifted,plotopts);
    axis equal
end








