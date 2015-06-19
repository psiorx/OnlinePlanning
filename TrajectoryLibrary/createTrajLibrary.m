addpath('../');
% Declare plane
p = SBachPlant_world();

% Trajectory library

% load ('../../Logs/initConditions_all_0.8.mat');
load ('../../Logs/initConditions_all.mat');

% x0 = mean(x0s_xhat(:,8:16),2);
x0 = mean(x0s_xhat,2); 

zero_ind = [1 2 3 4 6 8 10 12];
x0(zero_ind) = zeros(length(zero_ind),1); % return;

% Initial guess for duration
tf0 = .7;

obs_ys = -0.225; % [0 -0.1 -0.15 -0.17 -0.22];
% obs_ys = [-0.8 -0.75 -0.7 -0.65 -0.6 -0.55 0.55]; % -0.8

% load trajLibrary_May26.mat
% trajLibrary_old = trajLibrary;

% trajLibrary = cell(size(obs_ys));
load trajLibrary.mat


for k = 1 % length(trajLibrary)+1 % 1:length(trajLibrary)
        
    % Bounds on final state 
                %[X      Y       Z       roll   Pitch    yaw      U      V       W       P   Q   R]
    xf_low  = [ 4.0,  -3.0 ,   -0.1,    -0.4,   -0.4,   -0.6,    4.0,  -5.0,   -5.0 ,   -5, -5, -5]';
    xf_high = [ 4.5,   3.0,     0.100,     0.4,    0.4,    0.6,    7.5,   5.0,    5.0 ,    5,  5,  5]';
    
    
%     %input bounds
%     u_lb = [400;250+110;250+110;250+110];
%     u_ub = [700;850-110;850-110;850-110]; 
    
    %input bounds
    u_lb = [400;250+100;250+100;250+100];
    u_ub = [720;850-100;850-100;850-100]; 
    
    udot_lb = [-500;-2500;-2500;-2500]; 
    udot_ub = [750;2500;2500;2500]; 
    
%     udot_lb = [-800;-2500;-2500;-2500]; 
%     udot_ub = [800;2500;2500;2500]; 
    
    % generate an initial trajectory
    pts = 40; % 20
    u0 = 512*ones(4,pts) + 0*randn(4,pts);
    u0(1,:) = 600 + 0*randn(1,pts); % 400
    % u0(4,:) = 512 - 200;
    
    utraj0 = PPTrajectory(foh(linspace(0,tf0,pts),u0)); % return;
    
    % load utrajGuess.mat
    utraj0 = trajLibrary{k-0*1}.utraj;
    xtraj0 = trajLibrary{k-0*1}.xtraj; xtraj0 = xtraj0.setOutputFrame(p.getStateFrame);
    
    % utraj0 = trajLibrary_old{k}.utraj;
    
     utraj0 = utraj0.setOutputFrame(p.getInputFrame);
    
    
    
    disp('setting up constraints')
    con = struct();
    con.x0.lb = x0;
    con.x0.ub = x0;
    con.xf.lb = xf_low;
    con.xf.ub = xf_high;
    con.T.lb = 0.2;
    con.T.ub = 1.0;
    con.u.lb = u_lb;
    con.u.ub = u_ub;
    con.u_derivative.lb = udot_lb;
    con.u_derivative.ub = udot_ub;
    
%     con.u0.lb = [600;512;512;512]; % Make initial control input the trim
%     con.u0.ub = [600;512;512;512]; % Make initial control input the trim
    
    con.u0.lb = [600;512;512;512];
    con.u0.ub = con.u0.lb;



    
    con.x.lb = [-Inf -Inf -Inf -pi/2 -pi/2 -pi/2 -Inf -Inf -Inf -Inf -Inf -Inf]';
    con.x.ub = [Inf Inf Inf pi/2 pi/2 pi/2 Inf Inf Inf Inf Inf Inf]';
    
    % Add obstacle constraints
    R = 0.22 + 0.50 + 0*0.05; % Radius of airplane sphere + radius of obstacle
    obstacles(1).orientation = 'vertical';
    obstacles(1).xobs = [2.5-0.07;0.75+obs_ys(k)]; % 2.5 - 0.05
    
    obstacles(2).orientation = 'vertical';
    obstacles(2).xobs = [2.5-0.07;-1.25+obs_ys(k)];
    
%     obstacles(2).orientation = 'vertical';
%     obstacles(2).xobs = [1;0.7];

    con = addObstacleCon(con,obstacles,R); % orientation and x,y position of obstacle
    
    options = struct();
    options.method='dircol';
    options.grad_method = 'numerical';
    % options.xtape0 = 'simulate';
    options.xtape0 = xtraj0; % 'simulate';
    options.optimizeSparsity = false;
    
    %options.MajorOptimalityTolerance=1e-3; % 3e-1
    %options.MinorOptimalityTolerance=1e-3;
    % options.grad_test = true;
    
    % options.iterations = 100000;
    % snset('iterations',100000);
    % generate a trajectory
    disp('calling trajectoryOptimization')
    tic
    [utraj,xtraj,info] = trajectoryOptimization(p,@cost,@finalcost,x0,utraj0,con,options);info
    toc
   
    
    if info == 1
        trajLibrary{k}.xtraj = xtraj;
        trajLibrary{k}.utraj = utraj;
    end
    
end
    
    
    
    
    
    
    
    
    
    
    
