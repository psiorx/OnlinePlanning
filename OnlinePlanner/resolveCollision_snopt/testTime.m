% tic
% for k = 1:1000
% [x,F,info,xmul,Fmul] = snopt(x_guess,xlow,xupp,Flow,Fupp,'snoptDummyFun',0,1,[],[],[],[1;2;1;2;1;2],[1;1;2;2;3;3]);
% end
% toc

addpath('../');
tic
for k = 1:1000
    [a,b] = isCollisionFree_mex(x_shifted_full,forest,funnel,1);
    % nextFunnel = replanFunnels_mex(x0,forest,funnelLibrary);

end
toc
