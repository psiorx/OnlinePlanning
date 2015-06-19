function [C,Ceq] = collisionConstraintFmincon(x_shifted,funnel,forest)

Ceq = [];

% tic
[~,f,df] = collisionConstraintMex(x_shifted,forest,funnel,1); 
% toc
% df = df';

C = -f + 1;


end