close all
% % % Define ellipsoid (x'*S*x)
% x = 20*randn(12,1);
% S = sparse(x*x');
% S = full(S + 0.1*speye(12));
% s1 = 0.5*randn(1,12);
% % s2 = 0.1*randn(1);

t0 = rand(1);
S = V_pframe.S.eval(t0);
s1 = V_pframe.s1.eval(t0)';
s2 = V_pframe.s2.eval(t0);

x0 = -0.5*(S\s1');

% x0 = 10*randn(12,1);
% s1 = -2*x0'*S;
% s2 = x0'*S*x0;

% Projection matrix
C = [eye(3), zeros(3,9)];

Sp = inv(C*(S\C'));
s1p = -2*x0'*C'*Sp;
s2p = (C*x0)'*Sp*(C*x0);

% Plot ellipsoid
x = msspoly('x',12);
frame = CoordinateFrame('xframe',12);
V = QuadraticLyapunovFunction(frame,S,s1',s2);
options.inclusion = 'projection';
options.num_samples = 1000;
plotFunnel3(V,options);
axis equal

% Plot projected ellipsoid
% figure
y = msspoly('y',3);
framey = CoordinateFrame('yframe',3);
Vy = QuadraticLyapunovFunction(framey,Sp,s1p',s2p);

% figure
hold on
plotFunnel3(Vy,options);
axis equal
