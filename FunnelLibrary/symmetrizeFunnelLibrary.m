% Load library to be symmetrized
% load('funnelLibrary_Apr7.mat');
load('funnelLibrary_Apr24.mat');

% Declare coordinate transformation that does flip
original_frame = p.getStateFrame();
flipped_frame = CoordinateFrame('flipped_frame',12);
tr_flip = diag([1 -1 1 -1 1 -1 1 -1 1 -1 1 -1]);
% original_frame.addTransform(AffineTransform(original_frame,flipped_frame,tr_flip,zeros(12,1)));
% flipped_frame.addTransform(AffineTransform(flipped_frame,original_frame,tr_flip_back,zeros(12,1)));

tr_flip_inputs = diag([1 -1 1 -1]);

% Projection matrix
C = [eye(3), zeros(3,9)];

% Airplane radius: funnel is inflated by this amount (only the thing used
% for collision checking)
airplane_radius = 0.22; % m

clear funnelLibrary_flipped;
for k = 1:length(funnelLibrary)
    
    
%     if k == 2
%         funnelLibrary(k) = funnelLibrary(1);
%         funnelLibrary_flipped(k) = funnelLibrary_flipped(1);
%         continue;
%     end
    
    % Flip xtraj
    xtraj = funnelLibrary(k).xtraj;
    xtraj = xtraj.setOutputFrame(p.getStateFrame);
    xtraj_flipped = tr_flip*xtraj;
    xtraj_flipped = xtraj_flipped.setOutputFrame(p.getStateFrame);
    funnelLibrary_flipped(k).xtraj = xtraj_flipped;
    
    % Flip utraj
    utraj = funnelLibrary(k).utraj;
    utraj = utraj.setOutputFrame(p.getInputFrame);
    
    ts = utraj.getBreaks();
    us = utraj.eval(ts);
    us_flipped = us;
    us_flipped(2,:) = -1*(us(2,:) - 512) + 512; % Flip aileron commands
    us_flipped(4,:) = -1*(us(4,:) - 512) + 512; % Flip rudder commands
    utraj_flipped = PPTrajectory(foh(ts,us_flipped));
    utraj_flipped = utraj_flipped.setOutputFrame(p.getInputFrame);
    funnelLibrary_flipped(k).utraj = utraj_flipped;
    
    % Now flip controller
    tv = funnelLibrary(k).controller;
    ts = tv.y0.getBreaks();
    
    Ds = tv.D.eval(ts);
    y0s = tv.y0.eval(ts);
    
    Ds_flipped = Ds;
    y0s_flipped = y0s;
    for j = 1:size(Ds,3)
        Ds_flipped(:,:,j) = tr_flip_inputs*Ds(:,:,j)*tr_flip;
        y0s_flipped(:,:,j) = tr_flip_inputs*y0s(:,:,j) + 2*[0;512;0;512];
    end
    
    tv_flipped = AffineSystem([],[],[],[],[],[],[],PPTrajectory(spline(ts,Ds_flipped)),PPTrajectory(spline(ts,y0s_flipped)));
    tv_flipped = tv_flipped.setInputFrame(p.getStateFrame);
    tv_flipped = tv_flipped.setOutputFrame(p.getInputFrame);
    
    funnelLibrary_flipped(k).controller = tv_flipped;
    
    % Flip funnel
    V = funnelLibrary(k).V;
    
    ts = V.S.getBreaks();
    Ss = V.S.eval(ts);
    s1s = V.s1.eval(ts);
    s2s = V.s2.eval(ts);
    
    Ss_flipped = Ss;
    s1s_flipped = s1s;
    s2s_flipped = s2s;
    
    for j = 1:size(Ss,3)
        Ss_flipped(:,:,j) = tr_flip'*Ss(:,:,j)*tr_flip;
        s1s_flipped(:,:,j) = s1s(:,:,j)'*tr_flip;
        s2s_flipped(j) = s2s(j);
    end
    
    S_flipped = PPTrajectory(spline(ts,Ss_flipped));
    s1_flipped = PPTrajectory(spline(ts,s1s_flipped));
    s2_flipped = PPTrajectory(spline(ts,s2s_flipped));
    
    V_flipped = QuadraticLyapunovFunction(p.getStateFrame,S_flipped,s1_flipped,s2_flipped);
    
    funnelLibrary_flipped(k).V = V_flipped;
    
    % Now extract all the other stuff from xtraj, utraj, tv, V
    
    % ts
    funnelLibrary_flipped(k).ts = funnelLibrary_flipped(k).V.S.getBreaks();
    
    % Sp, s1p, s2p
    ts = funnelLibrary_flipped(k).ts;
    for j = 1:length(ts)
        
        S = funnelLibrary_flipped(k).V.S.eval(ts(j));
        s1 = funnelLibrary_flipped(k).V.s1.eval(ts(j))';
        s2 = funnelLibrary_flipped(k).V.s2.eval(ts(j));
        
        x0 = -0.5*(S\s1');
        
        Sp = inv(C*(S\C'));
        funnelLibrary_flipped(k).Sp{j} = Sp;
        funnelLibrary_flipped(k).s1p{j} = -2*x0'*C'*Sp;
        funnelLibrary_flipped(k).s2p{j} = (C*x0)'*Sp*(C*x0);
    end
    
    % x0
    x0 = funnelLibrary_flipped(k).xtraj.eval(ts);
    funnelLibrary_flipped(k).x0 = x0;
    
    % cS
    for j = 1:length(funnelLibrary_flipped(k).ts)
        Sp = funnelLibrary_flipped(k).Sp{j};
        
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
        
        funnelLibrary_flipped(k).cS{j} = chol(Sp2);
    end
    
    % xyz
    funnelLibrary_flipped(k).xyz = x0(1:3,:);
    
    % S0
    funnelLibrary_flipped(k).S0 = funnelLibrary_flipped(k).V.S.eval(0);
    
    
end

% return;
clear funnelLibrary_combined;
for k = 1:length(funnelLibrary)
    funnelLibrary_combined(2*k-1) = funnelLibrary(k);
    % funnelLibrary_new(2*k) = funnelLibrary_flipped(k);
    funnelLibrary_combined(2*k).controller = funnelLibrary_flipped(k).controller;
    funnelLibrary_combined(2*k).xtraj = funnelLibrary_flipped(k).xtraj;
    funnelLibrary_combined(2*k).ts = funnelLibrary_flipped(k).ts;
    funnelLibrary_combined(2*k).utraj = funnelLibrary_flipped(k).utraj;
    funnelLibrary_combined(2*k).V = funnelLibrary_flipped(k).V;
    funnelLibrary_combined(2*k).Sp = funnelLibrary_flipped(k).Sp;
    funnelLibrary_combined(2*k).s1p = funnelLibrary_flipped(k).s1p;
    funnelLibrary_combined(2*k).s2p = funnelLibrary_flipped(k).s2p;
    funnelLibrary_combined(2*k).x0 = funnelLibrary_flipped(k).x0;
    funnelLibrary_combined(2*k).cS = funnelLibrary_flipped(k).cS;
    funnelLibrary_combined(2*k).xyz = funnelLibrary_flipped(k).xyz;
    funnelLibrary_combined(2*k).S0 = funnelLibrary_flipped(k).S0;
end
    
















