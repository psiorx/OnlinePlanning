function [xtrajCam,xtrajSBach,q_cross] = getCameraTraj(file_name,tt_start)

% Load data
load(file_name);

if nargin < 2
    tt = Camera_x(:,1);
    tt_start = tt(1);
end

% % Format of data 
% #SBach_x  <class 'vicon_t.vicon_t'> :
% #[
% #1- timestamp
% #2- q(6)
% #8- log_timestamp
% #]
% 
% #Camera_x  <class 'vicon_t.vicon_t'> :
% #[
% #1- timestamp
% #2- q(6)
% #8- log_timestamp
% #]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get raw positions and orientations
t_q = Camera_x(:,8); % Log timestamp
q = Camera_x(:,2:7); % [x,y,z,roll,pitch,yaw]

for k = 1:length(q) % Convert to rpy
    ang = q(k,4:6);
    rpy = quat2rpy(angle2quat(ang(1),ang(2),ang(3),'XYZ'));
    q(k,4:6) = rpy;
end

% Flip signs on q to align axes
q(:,1) =  q(:,1); % X is the same
q(:,2) = -q(:,2); % Y is now to the right
q(:,3) = -q(:,3); % Z is down
q(:,4) =  q(:,4); % Roll is the same 
q(:,5) = -q(:,5); % Pitch gets sign flipped
q(:,6) = -unwrap(q(:,6)); % Yaw gets sign flipped

% Filter 
[b,a] = butter(1,0.01); % 0.01
q(:,1) = filtfilt(b,a,q(:,1));

[b,a] = butter(1,0.01); % 0.01
q(:,2) = filtfilt(b,a,q(:,2));

[b,a] = butter(1,0.005); % 0.007
q(:,3) = filtfilt(b,a,q(:,3));

[b,a] = butter(1,0.005);% 0.007
q(:,4) = filtfilt(b,a,q(:,4));

[b,a] = butter(1,0.003);% 0.007
q(:,5) = filtfilt(b,a,q(:,5));

[b,a] = butter(1,0.003);% 0.007
q(:,6) = filtfilt(b,a,q(:,6));


% q(:,4:6) = [sgolayfilt(q(:,4),3,41),sgolayfilt(q(:,5),3,41),sgolayfilt(q(:,6),3,41)];

% % Figure out where to start
% diff_z = diff(q(:,3))/(1/120);
% ind_start = find(diff_z < -0.1 & diff_z > -5);
% ind_start = ind_start(1); % 1310
tt = Camera_x(:,1);
ind_start = find(tt == tt_start);

t_q = t_q(ind_start:end) - t_q(ind_start);
q = q(ind_start:end,:);

% Make spline
% ts = 0:(1/fps):t_q(end);

% t_q = 0:(1/120):(length(t_q)-1)*(1/120);
xtrajCam = PPTrajectory(spline(t_q,q'));

%%% Now get SBach traj too %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get raw positions and orientations
t_q = SBach_x(:,8); % Log timestamp
q = SBach_x(:,2:7); % [x,y,z,roll,pitch,yaw]

for k = 1:length(q) % Convert to rpy
    ang = q(k,4:6);
    rpy = quat2rpy(angle2quat(ang(1),ang(2),ang(3),'XYZ'));
    q(k,4:6) = rpy;
end

% Flip signs on q to align axes
q(:,1) =  q(:,1); % X is the same
q(:,2) = -q(:,2); % Y is now to the right
q(:,3) = -q(:,3); % Z is down
q(:,4) =  q(:,4); % Roll is the same 
q(:,5) = -q(:,5); % Pitch gets sign flipped
q(:,6) = -q(:,6); % Yaw gets sign flipped

% Figure out when plane crossed x_cross
x_cross = -0.6;
[~, tcross_ind] = min(abs(q(:,1) - x_cross));
q_cross = q(tcross_ind,:)';

% Filter 
[b,a] = butter(1,0.2);
q(:,1:6) = filtfilt(b,a,q(:,1:6));

t_q = t_q(ind_start:end) - t_q(ind_start);
q = q(ind_start:end,:);

% t_q = linspace(0,t_q(end),length(t_q));

% Make spline
t_q = 0:(1/120):(length(t_q)-1)*(1/120);

xtrajSBach = PPTrajectory(spline(t_q,q'));


end




