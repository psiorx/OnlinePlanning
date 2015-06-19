% Define plane plant
addpath('../');
p = SBachPlant_world_vels();

load ../TrajectoryLibrary/trajLibrary.mat
% load funnelLibrary.mat
% load funnelLibrary_Jan17.mat
% load funnelLibrary_Feb06_2.mat
% load funnelLibrary_Feb10.mat

strs = {'18'};

for kk = 1:length(strs)

xtraj = trajLibrary{str2num(strs{kk})}.xtraj; % 
utraj = trajLibrary{str2num(strs{kk})}.utraj; % 

% xtraj = funnelLibrary(1).xtraj;
% utraj = funnelLibrary(1).utraj;

xtraj = xtraj.setOutputFrame(p.getStateFrame);
utraj = utraj.setOutputFrame(p.getInputFrame); % return;

% % Feb 10:
% % Do tvlqr
% Q = diag([2000 6000 1000, 500 500 500, 1200 700 500, 500 200 500]);
% R = 0.2*diag([0.1 0.1 0.5 0.1]); % 0.25
% Qf = Q;

load initialConditionSet_small.mat
% load G0_big.mat
% G0 = G0_big;


% March 23:
% Do tvlqr
Q = diag([2000 6000 1000, 500 500 500, 1200 700 500, 500 200 500]);
R = 0.2*diag([0.1 0.1 0.5 0.1]); % 0.25
Qf = G0*1000;



options = struct();
% options.sqrtmethod = false;

[tv,Vtv] = tvlqr(p,xtraj,utraj,Q,R,Qf,options); % return;


% Vtv = V;

% x = p.getStateFrame.poly;


sysCl = feedback(p,tv); 

disp('Doing taylor approx');
psys = taylorApprox(sysCl,xtraj,[],3); 

utraj = utraj.setOutputFrame(p.getInputFrame);
polyOrig = taylorApprox(p,xtraj,utraj,3); % return;


% Do frame thing
% p.getStateFrame.addTransform(AffineTransform(p.getStateFrame,p.getStateFrame,eye(length(x)),double(0*x)));

% Get time samples
ts = Vtv.S.getBreaks(); tsend = ts(end);
% ts = ts(1:ceil(ts(end)/12*(1/mean(diff(ts)))):length(ts));
ts = linspace(ts(1),ts(end),12);
% ts = [ts tsend];

ts = ts(1:end-1);

% Do verification
options = struct();
options.saturations = false;
options.rho0_tau = 10;
options.rho0 = 1;
options.degL1 = 2;
options.max_iterations = 10;
% options.solveroptions.OutputFlag = 0; 
disp('Starting verification...')

save(['funnelStuff' strs{kk} '.mat'])

load(['funnelStuff' num2str(str2num(strs{kk})-7) '_funnel.mat'], 'V');
% load('funnelLibrary_Feb10_funnel.mat','funnelLibrary');
% V = funnelLibrary(1).V;
% clear funnelLibrary

Vold = V;
for k = 1:length(ts)
    % Phi{k} = funnelLibrary(1).V.S.eval(ts(k)) - Vtv.S.eval(ts(k))/10000;
    Phi{k} = Vold.S.eval(ts(k)) - Vtv.S.eval(ts(k))/10000;
end

% Load initial condition set
% load initialConditionSet.mat

% computeInitialPhi;

% load Phis1.mat

[V,rho,Phi]=sampledFiniteTimeReach_B0(psys,polyOrig,Vtv/10000,G0,1000*G0,tv,ts,xtraj,utraj,options,Phi);
% [V,rho,Phi]=sampledFiniteTimeReach_B0(psys,polyOrig,Vtv/10000,G0,1000*G0,tv,ts,xtraj,utraj,options,Phi,rho);

V = V.setFrame(Vtv.getFrame);

save(['funnelStuff' strs{kk} '_funnel.mat'])

close all;

end

return;

% Convert V back to state frame and plot it
Vxframe = V.inFrame(p.getStateFrame());
figure
options.plotdims = [1 2];
options.x0 = xtraj;
options.ts = V.S.getBreaks;
options.inclusion = 'projection';
% options.inclusion = 'slice';
plotFunnel(Vxframe,options);
fnplt(xtraj,[1 2]); 
axis equal

% Tests to make sure simulated trajectories stay inside computed funnel
doTest = 1;
if doTest
    
% Create closed loop system with optimized controller
% sysCl = feedback(p,tv);
V0 = Vxframe.getPoly(0); % Get inlet of funnel
opts = struct();
opts.x0 = zeros(12,1);
opts.num_samples = 100;
xinit = getLevelSet(decomp(V0),V0,opts);

% xp = psys.getStateFrame.poly;
% up = psys.getInputFrame.poly;

addpath('/home/anirudha/Documents/Research/MURI/SBach/OnlinePlanning/Util');

Vsall = [];
figure
hold on
grid on
for j = 1:length(xinit)
    Vs = [];
    x0 = 0.99*xinit(:,j); % + xtraj.eval(0); % Simulate from 0.95*boundary of funnel
    % xsim = psys.simulate([0 ts(end)],x0);
    % xsim = simulateEuler(psys,ts(end),x0,0.001,'msspoly',xp,up);
    xsim = sysCl.simulate([0 ts(end)],x0);
    for k = 1:length(ts)
        Vs = [Vs, Vxframe.eval(ts(k),xsim.eval(ts(k)))];
    end
    Vsall = [Vsall, Vs];
    
    plot(ts,Vs)
    plot(ts,ones(1,length(ts)),'ro')
    drawnow;
end
end















