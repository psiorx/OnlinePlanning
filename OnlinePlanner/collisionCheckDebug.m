close all
% Define ellipsoid (x'*S*x)
x = 2*randn(12,1);
S = sparse(x*x');
S = full(S + 0.1*speye(12));
s1 = 1*randn(1,12); 
x0 = -0.5*(S\s1');
s2 = x0'*S*x0;

% Get projected ellipsoid
% Projection matrix
C = [eye(3), zeros(3,9)];

Sp = inv(C*(S\C'));
s1p = -2*x0'*C'*Sp;
s2p = (C*x0)'*Sp*(C*x0);

% % Define obstacle (extruded triangle)
theta_range = 10*pi/180; % radians
dist_range = 0.1;
    
theta1 = 135*pi/180 + (2*rand(1)-1)*theta_range;
n1 = [cos(theta1) sin(theta1) 0];

theta2 = 225*pi/180 + (2*rand(1)-1)*theta_range;
n2 = [cos(theta2) sin(theta2) 0];

theta3 = 0*pi/180 +  (2*rand(1)-1)*theta_range;
n3 = [cos(theta3) sin(theta3) 0];

% Add last two inequalities just for plotting
Aineq = [n1;n2;n3];
bineq = dist_range*ones(3,1) + Aineq*(1*randn(3,1));
    

% Plot ellipsoid
x = msspoly('x',12);
frame = CoordinateFrame('xframe',12);
V = QuadraticLyapunovFunction(frame,S,s1',s2);
options.inclusion = 'projection';
options.num_samples = 1000;
plotFunnel3(V,options);

% Plot polytope
P = polytope([Aineq;0 0 -1;0 0 1],[bineq;20;30]);
hold on
plot(P)
view(3)

% Projection matrix
C = [eye(3), zeros(3,9)];

% Cvxgen code
settings.verbose = 0;
params.A = Aineq;
params.b = bineq;
params.S = Sp;
params.s1 = s1p;
params.s2 = s2p;

tic;
[vars, status] = csolve(params,settings);
toc

collision = status.optval < 1