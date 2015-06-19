function [f,df] = collisionConstraint(x_shifted,funnel,forest)


% tic
[~,f,df] = collisionConstraintMex(x_shifted,forest,funnel,1); 
% toc
df = df';


end