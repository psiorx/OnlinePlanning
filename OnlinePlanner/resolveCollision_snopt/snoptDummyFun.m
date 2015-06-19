function [f,G] = snoptDummyFun(x)

global SNOPT_USERFUN;
%try % snopt crashes if the userfun crashes
[f,G] = SNOPT_USERFUN(x);
% G=full(G);  % it seems that snopt can't handle sparse matrices.
