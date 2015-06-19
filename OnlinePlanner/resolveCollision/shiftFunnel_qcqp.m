function [x_opt,collFree,tau_opt] = shiftFunnel_qcqp(x_current,forest,funnelLibrary,funnelIdx)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

% Get funnel
funnel = funnelLibrary(funnelIdx);
N = length(funnel.ts);
x0s = funnel.xyz;
x0 = funnel.x0(:,1);

v = x0(4:end) - x_current(4:end); % Difference in non-cyclic dimensions

% Setup mosek QCQP problem

% Specify linear objective term
prob.c = [0;0;0;1];

% Specify linear terms in constraints
S0 = funnel.S0;
S11 = S0(1:3,1:3);
% S12 = S0(1:3,4:end); This is just S21'
S21 = S0(4:end,1:3);
S22 = S0(4:end,4:end);

%%%%%
% load debugStuff.mat
% prob.a = [L1(1:3)+L2,0];
%%%%%

L1 = 2*v'*S21;

prob.a = [L1,0];

% Now initialize upper bounds
C = v'*S22*v;

prob.buc = 1-C;

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
        prob.a = [prob.a;[-ns_jk,-1]];
        
        prob.buc = [prob.buc;ds_jk];
        
        
    end
end

% Constrain tau >= 0
prob.a = [prob.a;0,0,0,-1];
prob.buc = [prob.buc;0];

% Specify quadratic terms of the constraints
prob.qcsubi = [1;2;3;2;3;3];
prob.qcsubj = [1;1;1;2;2;3];
prob.qcval = 2*[S11(1,1);S11(2,1);S11(3,1);S11(2,2);S11(3,2);S11(3,3)];

prob.qcsubk = [1;1;1;1;1;1]; % we only have quadratic terms in the first constraint



% Call mosek to solve
[~,res] = mosekopt('minimize echo(0)',prob); 


% Get results. Shift result into absolute coordinates. x_opt is absolute
% coordinates of shifted funnel start position.
x_opt = res.sol.itr.xx(1:3) + x_current(1:3);

% See if we are collision free after shift
collFree = (res.sol.itr.xx(4) <  1e-4); % If tau is 0, we are collision free

tau_opt = res.sol.itr.pobjval;


end




