%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate code for QCQP using Forces Pro.
% 
% Sets up a QCQP to try to shift a funnel out of collision while keeping the
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get funnel
funnel = funnelLibrary(funnelIdx);
N = length(funnel.ts);
x0s = funnel.xyz;
x0 = funnel.x0(:,1);

% Setup forces QCQP problem
stages = MultistageProblem(1);

stages(1).dims.n = 4; % Number of decision variables
stages(1).dims.r = 0; % Number of equality constraints
stages(1).dims.l = 0; % Number of lower bounds
stages(1).dims.u = 0; % Number of upper bounds
stages(1).dims.p = N*length(forest)+1; % N*length(forest)+1; % Number of polytopic constraints
stages(1).dims.q = 1; % Number of quadratic constraints 

% Specify objective term
stages(1).cost.H = zeros(4,4);
stages(1).cost.f = [0;0;0;1];

% % Specify linear terms in constraints
S0 = funnel.S0;
S11 = S0(1:3,1:3);
% S12 = S0(1:3,4:end); This is just S21'
S21 = S0(4:end,1:3);
S22 = S0(4:end,4:end);

stages(1).ineq.p.A = []; 

% Now initialize upper bounds
stages(1).ineq.p.b = []; 

for j = 1:length(forest)
    for k = 1:N
        % Compute shift required
        forest_jk = forest{j} - repmat(x0s(:,k) + x_current(1:3),1,size(forest{j},2));
        cS = funnel.cS{k};
        forestk_tr{1} = cS*forest_jk;
        [dk_tr,nk_tr] = testResolveCollision(forestk_tr);
        
        ns_jk = nk_tr'*cS;
        ds_jk = dk_tr; % - ns_jk*(x_current(1:3));
        
        % Append to linear constraints and upper bounds
        stages(1).ineq.p.A = [stages(1).ineq.p.A;[-ns_jk,-1]];
        
        stages(1).ineq.p.b = [stages(1).ineq.p.b;ds_jk];
        
        
    end
end

% Constrain tau >= 0
stages(1).ineq.p.A = [stages(1).ineq.p.A;0,0,0,-1];
stages(1).ineq.p.b = [stages(1).ineq.p.b;0];

% Specify quadratic constraint
stages(1).ineq.q.idx = {[1 2 3]};
stages(1).ineq.q.Q = {S11};

% Call Forces to generate code
% Parameters
clear params;
params(1) = newParam('A',1,'ineq.p.A'); % 
params(2) = newParam('b',1,'ineq.p.b'); % 
params(3) = newParam('ql',1,'ineq.q.l',1); %
params(4) = newParam('qr',1,'ineq.q.r',1); % 
% params(5) = newParam('Q',1,'ineq.q.Q',1); % 

% Define outputs of the solver
outputs(1) = newOutput('x_shift',1,1:3); % solver output
outputs(2) = newOutput('tau',1,4);

% Solver settings
% codeoptions = getOptions('FORCESPro_Reference_Tracking');
codeoptions = getOptions();
% codeoptions.name = 'funnel31_shift';
codeoptions.name = ['funnel' ind_strs{ind} '_shift'];
codeoptions.maxit = 50;
codeoptions.printlevel = 0; % 2
codeoptions.overwrite = 1; % 1
codeoptions.init = 0;

% Generate code
generateCode(stages,params,codeoptions,outputs);









