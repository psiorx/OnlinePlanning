x = x0 + 5*randn(12,1);

%forest = op.forestAhead.getData();
%fL = op.funnelLibraryReduced;
tic; for k = 1:1000
    % nextFunnel = replanFunnels_mex(x0,forest,fL);
     nextFunnel = op.replanFunnels(0, x0); 
end; toc

return;
tic; for k = 1:1000
    dd = xrel'*S0*xrel;
end; toc

% tic;
% t = 0;
% for k = 1:1000
%     t = t + 0.5;
%      op.output(t,[],x); 
% end; 
% toc
return;
for k = 1:1000
    navigateForest;
    x = x0 + 5*randn(12,1);
    isFree = op.isCollisionFree(1,x);
    isFreeOld = op.isCollisionFreeOld(1,x);
    
    if isFree ~= isFreeOld
        error('Did not match.');
    end
end



