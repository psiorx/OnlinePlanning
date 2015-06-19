function [g,dg] = cost(t,x,u)
R = eye(length(u));
% R(1,1) = 100;
% R(2,2) = 100;
% R(3,3) = 100;
% R(4,4) = 100;

R(1,1) = 0.001;
R(2,2) = 0.001;
R(3,3) = 0.001;
R(4,4) = 0.001;
u0 = [250;512;512;512];
% u0 = [250;0;0;0];
g = (u-u0)'*R*(u-u0);
dg = [zeros(1,1+size(x,1)),2*(u-u0)'*R];
%dg = zeros(1, 1 + size(x,1) + size(u,1));
%disp('costCalled');
end
 
