function [x_opt,collFree,tau_opt,exitflag,info] = shiftFunnel_qcqp_forces(x_current,forest,funnelLibrary,funnelIdx)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Same as shiftFunnel_qcqp, but calls Forces Pro solver instead.
% 
% Uses a QCQP to try to shift a funnel out of collision while keeping the
% current state in the inlet of the (shifted) funnel.
% 
% Sets up and solves the following optimization problem:
% min   max(0,tau)
% 
% s.t.  (x_current-x_shifted_full)^T S0 (x_current-x_shifted_full) <= 1
%       tau + ns_jk*x_shiftl(1:3) + ds_jk >= 0, for all j,k
%
% Here, the decision variables are x_shift. x_shifted_full is:
% [x_shift+x_current(1:3);x0(4:end)] (x0 is funnel.x0(:,1)).
%
% ns_jk and ds_jk are the collision normal and penetration distance
% respectively. The indices j and k iterate over obstacles and time-steps
% in the funnel respectively.
% 
% Returns the start position of the shifted funnel (x_opt) and also returns
% whether we succesfully shifted the funnel out of collision.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Check number of obstacles
maxObs = 20;
numObs = length(forest);
if numObs > maxObs;
    error('Maximum number of obstacles is 20. If you want more than 20 obstacles, regenerate forces code');
end


% Get funnel
funnel = funnelLibrary(funnelIdx);
N = length(funnel.ts);
x0s = funnel.xyz;
x0 = funnel.x0(:,1);
v = x0(4:end) - x_current(4:end); % Difference in non-cyclic dimensions

% Specify linear terms in constraints
S0 = funnel.S0;
% S11 = S0(1:3,1:3);
S12 = S0(1:3,4:end); % This is just S21'
% S21 = S0(4:end,1:3);
S22 = S0(4:end,4:end);

A_ineq = []; 

% Now initialize upper bounds
b_ineq = []; 

for j = 1:numObs
    for k = 1:N
        % Compute shift required
        forest_jk = forest{j} - repmat(x0s(:,k) + x_current(1:3),1,size(forest{j},2));
        cS = funnel.cS{k};
        forestk_tr{1} = cS*forest_jk;
        [dk_tr,nk_tr] = testResolveCollision(forestk_tr);
        
        ns_jk = nk_tr'*cS;
        ds_jk = dk_tr; % - ns_jk*(x_current(1:3));
        
        % Append to linear constraints and upper bounds
        A_ineq = [A_ineq;[-ns_jk,-1]];
        
        b_ineq = [b_ineq;ds_jk];
        
        
    end
end

% Constrain tau >= 0
A_ineq = [A_ineq;0,0,0,-1];
b_ineq = [b_ineq;0];

% Pad A and b to make sure we have N*length(forest)+1 constraints
pad_length = N*(maxObs - numObs);
A_ineq = [A_ineq;zeros(pad_length,4)];
b_ineq = [b_ineq;zeros(pad_length,1)];

problem.A = A_ineq;
problem.b = b_ineq;

% Specify quadratic constraint
L1 = 2*S12*v; % 2*v'*S21;

C = v'*S22*v;

% problem.Q = S11;
% problem.q_idx = {[1 2 3]};
problem.ql = L1;
problem.qr = 1-C;

% Call Forces
% [res,exitflag,info] = funnel1_shift(problem);
fn = str2func(['funnel' num2str(funnelIdx) '_shift']);
[res,exitflag,info] = fn(problem);

% Get x_opt from result
x_opt = res.x_shift + x_current(1:3);

% Check if we are collision free after shift
collFree = (res.tau < 1e-4); % If tau is 0, we are collision free
tau_opt = res.tau;

% xtau_opt = [x_opt;tau_opt];

end







