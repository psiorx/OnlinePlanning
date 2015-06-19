close all;
% Read video
vid = VideoReader('./rawFunnelMovies/video_2015_02_16_05.MP4'); 

% Figure out when camera moves by looking at frames
startFrame = 325; % computeStartFrame(vid,2,3);

% Now get camera positions starting when the camera is jerked up at the
% beginning
[xtrajCam,xtrajSBach] = getCameraTraj('../../../Logs/Feb162015/lcmlog_2015_02_16_05.mat');

% Load funnel
% load('../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');
load('./funnelSphere.mat');

funnel = funnelLibrary(1);

% Starting position of funnel
funnelPos = [-0.6;0;-1.54];

% Undistort using gopro calibration parameters
load('./goProCalibration/cameraParams_1080_120_N.mat');

% Number of frames in video
N = vid.NumberOfFrames;

% 
% % Initialize funnel video object
% funnelVideo = VideoWriter('./funnelMovies/funnelMovie_test.avi', 'Uncompressed AVI');
% 
% % Change frame rate to frame rate of original video
% funnelVideo.FrameRate = vid.FrameRate;
% fps = vid.FrameRate;
% 
% % Open video for writing
% open(funnelVideo);

% Make funnel image once at the beginning
% Get position of camera
camPosition = [0;0;0];
% Camera orientation
camOrientation = [1;0;0];
% "Up" vector of camera
camUp = [0;0;-1];
funnelPos = [2.959028530984140;-0.474267055818099;-0.085492699174567];
[~,figNum] = makeFunnelImage(funnel,funnelPos,camPosition,camOrientation,camUp,cameraParameters);



funnelVideo = avifile('./funnelMovies/funnelMovie_test.avi');
funnelVideo.Quality = 100;
funnelVideo.Compression = 'None';

fps = vid.FrameRate;
funnelVideo.Fps = fps;

% Read all images from video
startInd = 120*7;
endInd = 120*10;
ims = read(vid,[startInd endInd]);

for k = 1:(endInd-startInd+1)
        
    % Display frame number being processed
    disp(['Processing frame ' num2str(k) ' of ' num2str(endInd-startInd+1) ' ...']);
    
    % Close all windows
    % close all;
    
    % Load frame as image
    % im = read(vid,k);
    im = ims(:,:,:,k);
    
    % Undistort images using go pro camera params
    im_undistorted = undistortImage(im, cameraParameters);
    
    % Get position of camera
    % camPosition = [-0.5;0;-0.5];
    tk = (k+startInd-startFrame-1)*(1/fps);
    xCam = xtrajCam.eval(tk);
    
    % Camera orientation
    % camOrientation = [1;0;0];
    R_body_to_world = rpy2rotmat(xCam(4:6));
    xvec = [1;-0.03;-0.02];
    xvec = xvec/norm(xvec);
    camOrientation = R_body_to_world*xvec; % Orientation of x axis
    
    camPosition = xCam(1:3);
    % camPosition = camPosition + R_body_to_world*[0;-0.01;0];
    
    % "Up" vector of camera
    % camUp = [0;0;-1];
    camUp = R_body_to_world*[0;0;-1];
    
    % Get SBach position
    funnelPos = xtrajSBach.eval(tk);
    funnelPos = funnelPos(1:3);
        
    % im_funnel = makeFunnelImage(funnel,funnelPos,camPosition,camOrientation,camUp,cameraParameters);
    im_funnel = changeView(figNum,camPosition,camOrientation,camUp,cameraParameters);

    
    % Superimpose funnel image onto image from video
    F = superImposeFunnel(im_undistorted,im_funnel,0.7);
    
%     % Write frame to video using VideoWriter
%     writeVideo(funnelVideo,F);
    
    % Write frame to video using avifile
    funnelVideo = addframe(funnelVideo,F);
    
    
end

% close(funnelVideo);

% Close video object
funnelVideo = close(funnelVideo);





