load funnelLibrary.mat

% Projection matrix
C = [eye(3), zeros(3,9)];

ts = funnelLibrary{1}.ts;
for k = 1:length(ts)
    
    S = funnelLibrary{1}.V.S.eval(ts(k));
    s1 = funnelLibrary{1}.V.s1.eval(ts(k))';
    s2 = funnelLibrary{1}.V.s2.eval(ts(k));
    
    x0 = -0.5*(S\s1');

    Sp = inv(C*(S\C'));
    funnelLibrary{1}.Sp{k} = Sp;
    funnelLibrary{1}.s1p{k} = -2*x0'*C'*Sp;
    funnelLibrary{1}.s2p{k} = (C*x0)'*Sp*(C*x0);
end

