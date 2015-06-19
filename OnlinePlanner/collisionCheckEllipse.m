% Define ellipsoid (x'*S*x)
x = randn(12,1);
S = sparse(x*x');
S = S + 0.1*speye(12);
s1 = randn(12,1);
s2 = 0.001*randn(1);
% S = sparse(diag(rand(12,1)));
% S = speye(12);

% Define obstacle (extruded triangle)
Aineq = [1  1 0;
        -2 -1 0;
         1  -1  0];

% Aineq = rand(3,3);

Aineq = [Aineq, zeros(3,9)]; 
    
    
bineq = [10;-10;-7];
c = zeros(12,1);

% Setup QP
model.obj = c;
model.Q = S;
model.A = sparse(Aineq);
model.rhs = bineq;
model.sense = repmat('<',1,size(Aineq,1));
model.lb = -Inf*ones(size(Aineq,2),1);

options = struct();
options.outputflag = 0;

tic
output = gurobi(model,options);
toc

% Mosek version (much slower!)
% res = mskqpopt(2*S,c,Aineq,[],bineq); %,blx,bux,param,cmd)

% Cvxgen (Fastest!)
P = chol(S);
iP = inv(P);

params.A = Aineq/P;
params.b = bineq;
settings.verbose = 0;

tic;
[vars, status] = csolve(params,settings);
toc





