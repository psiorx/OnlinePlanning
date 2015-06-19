megaclear;
close all;
addpath('../../Visualization/');
addpath('../');
addpath('../../');

%%%%% Set test options%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
generate_code = true;
numExperiments = 100; % Number of random environments to test on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% randn('state',13);

%%% Setup obstacles, etc. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load funnel library
load('../../FunnelLibrary/funnelLibrary_Apr24_symmetrized.mat');

clear forest;

numObs = 20; % randi(20);
% Generate random obstacles
for k = 1:numObs
    numVerts = randi(10) + 5;
    forest{k} = randn(3,numVerts);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


ind_strs = {'1'}; %  '2' '3' '4' '5' '6' '7' '8' '9' '10' '11' '12' '13' '14' '15' ...
    % '16' '17' '18' '19' '20' '21' '22' '23' '24' '25' '26' '27' '28' ...
    % '29' '30' '31' '32' '33' '34' '35' '36' '37' '38' '39' '40'};

for ind = 1:length(ind_strs)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Specify current position of airplane and funnel number
    funnelIdx = str2num(ind_strs{ind});
    funnel = funnelLibrary(funnelIdx);
    x0 = funnel.x0(:,1);
    x_current = x0;
    x_current(1:3) = x_current(1:3) + mean(forest{1},2) + randn(3,1);
    x_current(4:end) = x_current(4:end) + 0.03*randn(9,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Generate Forces Pro code if needed
    shiftFunnel_generate_code;
end

