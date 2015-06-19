addpath('../');

load funnelStuff15_funnel.mat
% load funnelLibrary_Feb10_funnel.mat

% p = SBachPlant_world_vels();


% Projection matrix
C = [eye(3), zeros(3,9)];

% Airplane radius: funnel is inflated by this amount (only the thing used
% for collision checking)
airplane_radius = 0.22; % m

for k = 20 % :length(xtraj)
    
    % funnelLibrary_new(k) = funnelLibrary(1); continue;
    
    % Do frame stuff first
    xtraj = xtraj.setOutputFrame(p.getStateFrame);
    utraj = utraj.setOutputFrame(p.getInputFrame);
    
    % controller
    V = V.inFrame(p.getStateFrame());
    tv = tv.inInputFrame(p.getStateFrame);
    tv = tv.inOutputFrame(p.getInputFrame);
    funnelLibrary_new(k).controller = tv;
    
    % xtraj
    funnelLibrary_new(k).xtraj = xtraj;
    
    % ts
    funnelLibrary_new(k).ts = V.S.getBreaks();
    % funnelLibrary_new(k).ts = linspace(funnelLibrary_new(k).ts(1),funnelLibrary_new(k).ts(end),10);
    
    % utraj
    funnelLibrary_new(k).utraj = utraj;
    
    % V
    funnelLibrary_new(k).V = V;
    
    % Sp, s1p, s2p
    ts = funnelLibrary_new(k).ts;
    for j = 1:length(ts)
        
        S = funnelLibrary_new(k).V.S.eval(ts(j));
        s1 = funnelLibrary_new(k).V.s1.eval(ts(j))';
        s2 = funnelLibrary_new(k).V.s2.eval(ts(j));
        
        x0 = -0.5*(S\s1');
        
        Sp = inv(C*(S\C'));
        funnelLibrary_new(k).Sp{j} = Sp;
        funnelLibrary_new(k).s1p{j} = -2*x0'*C'*Sp;
        funnelLibrary_new(k).s2p{j} = (C*x0)'*Sp*(C*x0);
    end
    
    % x0
    x0 = xtraj.eval(ts);
    funnelLibrary_new(k).x0 = x0;
    
    % cS
    for j = 1:length(funnelLibrary_new(k).ts)
        Sp = funnelLibrary_new(k).Sp{j};
        
        % Inflate funnel (just the thing used for collision checking) by
        % airplane radius
        r = airplane_radius;
        [VV,DD] = eig(Sp);
        l1 = 1/sqrt(DD(1,1)) + r;
        l2 = 1/sqrt(DD(2,2)) + r;
        l3 = 1/sqrt(DD(3,3)) + r;
        d1 = 1/l1^2;
        d2 = 1/l2^2;
        d3 = 1/l3^2;
        D2 = diag([d1,d2,d3]);
        Sp2 = VV*D2*VV';
        
        funnelLibrary_new(k).cS{j} = chol(Sp2);
    end
    
    % xyz
    funnelLibrary_new(k).xyz = x0(1:3,:);
    
    % S0
    funnelLibrary_new(k).S0 = funnelLibrary_new(k).V.S.eval(0);
    
    
end

% funnelLibrary(k) = funnelLibrary_new(k);

