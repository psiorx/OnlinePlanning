function [f,df] = snoptWrapperFun(x_shift,funnel,forest,x_current)

% Initialize f and df
f = zeros(2,1);
df = zeros(2,3);

x_shifted = x_shift + x_current(1:3);

% Cost function (first row)
[~,d,n] = collisionConstraintMex(x_shifted,forest,funnel,1); 
% f_nom = f(1);

% Cost function:
% This cost function pulls d to d_ideal. That way, we're not always pushing
% the funnel way out, but we're still leaving some margin from the
% obstacle.
d_ideal = 1.1;
f(1) = d*(d-2*d_ideal);
df(1,:) = 2*d*n - 2*d_ideal*n; %


% Second row of f is containment constraint, f(2) <= 1
x0 = funnel.x0(:,1);
S0 = funnel.S0;
v = x0(4:end) - x_current(4:end);

xdiff = [x_shift;v];
f(2) = xdiff'*S0*xdiff;

df2 = 2*xdiff'*S0;
df(2,:) = df2(1:3);





end


