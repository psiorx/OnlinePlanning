addpath('../OnlinePlanner/');
addpath('../OnlinePlanner/resolveCollision/');
addpath('../OnlinePlanner/resolveCollision_snopt/');

% First get symmetrized library
symmetrizeFunnelLibrary;

N = length(funnelLibrary);

% Check that sizes are same
if N ~= length(funnelLibrary_flipped)
    error('Size of funnel library is different from the flipped library');
end

funnelInd = randi(N);

% Flip transformations
tr_flip = diag([1 -1 1 -1 1 -1 1 -1 1 -1 1 -1]);
tr_flip_inputs = diag([1 -1 1 -1]);

% % Check that simulating utraj open loop gives same results (upto flip) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% utraj = funnelLibrary(funnelInd).utraj;
% utraj = utraj.setOutputFrame(p.getInputFrame);
% utraj_flipped = funnelLibrary_flipped(funnelInd).utraj;
% 
% x0 = [0;0;0;0;0;0;6;0;0;0;0;0] + 0.1*randn(12,1);
% x0_flipped = tr_flip*x0;
% 
% sysOl = cascade(utraj,p);
% sysOl_flipped = cascade(utraj_flipped,p);
% 
% ts = funnelLibrary(funnelInd).ts;
% xtrajSim = sysOl.simulate([0 ts(end)], x0);
% xtrajSim_flipped = sysOl_flipped.simulate([0 ts(end)], x0_flipped);
% 
% for j = 1:length(ts)
%     if (any(abs(tr_flip*xtrajSim.eval(ts(j)) - xtrajSim_flipped.eval(ts(j))) > 1e-3))
%         error('Simulating utrajs open loop did not result in symmetric trajectories');
%     end
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Simulate tvlqr systems
% 
% tv = funnelLibrary(funnelInd).controller;
% tv = tv.setOutputFrame(p.getInputFrame);
% tv = tv.setInputFrame(p.getStateFrame);
% tv_flipped = funnelLibrary_flipped(funnelInd).controller;
% 
% sysCl = feedback(p,tv);
% sysCl_flipped = feedback(p,tv_flipped);
% 
% ts = funnelLibrary(funnelInd).ts;
% x0 = [0;0;0;0;0;0;6;0;0;0;0;0] + 0.1*randn(12,1);
% x0_flipped = tr_flip*x0;
% 
% xtrajSim = sysCl.simulate([0 ts(end)], x0);
% xtrajSim_flipped = sysCl_flipped.simulate([0 ts(end)], x0_flipped);
% 
% for j = 1:length(ts)
%     if (any(abs(tr_flip*xtrajSim.eval(ts(j)) - xtrajSim_flipped.eval(ts(j))) > 1e-3))
%         error('Simulating controllers closed loop did not result in symmetric trajectories');
%     end
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now check that collion checking is symmetric

numExps = 100;
for exp = 1:numExps
    
    % Positions of poles
    clear forest forest_flipped;
    pos = 2*randn(4,3); % [3.0,3.0;-0.75,0.75;0.0,0.0]';
    A = [eye(3);-eye(3)];
    d = 0.025;
    
    % Generate poles
    for k = 1:size(pos,1)
        b = [pos(k,1)+d;pos(k,2)+d;0;d-pos(k,1);d-pos(k,2);2];
        P = polytope(A,b);
        forest{k} = extreme(P)';
    end
    
    % Flip forest
    for k = 1:length(forest)
        forest_flipped{k} = tr_flip(1:3,1:3)*forest{k};
    end
    
    % % Plot stuff
    % figure
    % hold on
    % for k = 1:length(forest)
    %     P = polytope(forest{k}');
    %     plot(P,'g');
    %     P = polytope(forest_flipped{k}');
    %     plot(P,'r');
    % end
    
    x_current = funnelLibrary(1).x0(:,1) + 0.01*randn(12,1);
    x_current_flipped = tr_flip*x_current;
    
    options = struct();
    % options.shift_method = 'qcqp'; % Don't use this here because this relies
    % on having the correct mex functions.
    options.penetration_thresh = -10.0;
    options.failsafe_penetration = -10.0;
    
    [nextFunnel,x_execute_next,collFree,min_dist] = replanFunnels_mex(x_current,forest,funnelLibrary,options);
    
    [nextFunnel_flipped,x_execute_next_flipped,collFree_flipped,min_dist_flipped] = replanFunnels_mex(x_current_flipped,forest_flipped,funnelLibrary_flipped,options);
    
     nextFunnel
%     if nextFunnel == 2
%         % counter = counter + 1;
%         continue;
%     end
    
    if nextFunnel ~= nextFunnel_flipped
        error('Replanning with flipped library did not yield same result');
    end
    
    if (any(abs(tr_flip(1:3,1:3)*x_execute_next - x_execute_next_flipped) > 1e-3))
        error('Simulating controllers closed loop did not result in symmetric trajectories');
    end
    
    if collFree ~= collFree_flipped
        error('collFree flag is not same for flipped library');
    end
    
    if abs(min_dist - min_dist_flipped) > 1e-3
        error('Penetration distance is not same with flipped library');
    end
    
end




















