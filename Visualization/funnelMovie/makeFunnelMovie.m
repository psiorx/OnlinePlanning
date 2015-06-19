close all;

% % File extension (corresponds to lcm logs)
% file_num = '2015_02_24_22';
% 
% % Read video
% vid = VideoReader(['./rawFunnelMovies/video_' file_num '.MP4']);
% 
% % Undistort using gopro calibration parameters
% load('./goProCalibration/cameraParams_1080_120_W.mat');
% 
% % Figure out when camera moves by looking at frames
% % startFrame = computeStartFrame(vid,2,3);
% % Show 49th,50th,51st frames in video to read time stamp off of
% im = read(vid,49);
% figure(49); imshow(undistortImage(im, cameraParameters));
% im = read(vid,50);
% figure(50); imshow(undistortImage(im, cameraParameters));
% im = read(vid,51);
% figure(51); imshow(undistortImage(im, cameraParameters));
% 
% startFrame = input(['Type in start frame: ']);
% tt_start = input(['Type in time stamp: ']);
% 
% close all;

% Now get camera positions
[xtrajCam,xtrajSBach] = getCameraTraj(['../../../Logs/Feb242015/lcmlog_' file_num '.mat'],tt_start);
% return;
% Load funnel
load('../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');
% load('./funnelSphere.mat');

funnel = funnelLibrary(1);

% X-vector of camera
% xvec = [1;-0.03;-0.02];
xvec = [1;0;0];
xvec = xvec/norm(xvec);

% Number of frames in video
N = vid.NumberOfFrames;

% Make funnel image once at the beginning
% Camera orientation
camOrientation = [1;0;0];
% "Up" vector of camera
camUp = [0;0;-1];
% Starting position of funnel
% funnelPos = xtrajSBach.eval(0); % [-0.6;0;-1.54;zeros(9,1)];
% funnelPos = funnelPos(1:3);
funnelPos = [-0.6;0;-1.54;zeros(9,1)]; % -1.43

% Get position of camera
camPosition = funnelPos(1:3) + [-0.5;0;-0.5];
[~,figNum] = makeFunnelImage(funnel,funnelPos,camPosition,camOrientation,camUp,cameraParameters);
% return;

% % Initialize funnel video object
% funnelVideo = VideoWriter('./funnelMovies/funnelMovie_test.avi', 'Uncompressed AVI');
%
% % Change frame rate to frame rate of original video
% funnelVideo.FrameRate = vid.FrameRate;
%
% % Open video for writing
% open(funnelVideo);

funnelVideo = avifile(['./funnelMovies/funnelMovie_' file_num  '.avi']);
funnelVideo.Quality = 100;
funnelVideo.Compression = 'None';

fps = vid.FrameRate;
funnelVideo.Fps = fps;

% Read all images from video
startTime = 42; % 42
endTime = 50; % 54 
startInd = startFrame + startTime*120;
endInd = startFrame + endTime*120 ; 

% Break this window into 7 second chunks (because if we try to read more
% than that at a time, my computer freezes)
numFrames = endInd-startInd+1;
chunkSize = floor(fps*7); % about 7 seconds at 120 fps
numChunks = floor(numFrames/chunkSize) + 1;
chunkInds = startInd + [0:chunkSize:(numChunks-1)*chunkSize];
chunkInds = [chunkInds, endInd];


for chunk = 1:length(chunkInds)-1
    
    startInd = chunkInds(chunk);
    endInd = chunkInds(chunk+1)-1;
    
    disp('Reading frames from video...');
    ims = read(vid,[startInd endInd]);
    disp('Done reading frames.');
    
    for k = 1:(endInd-startInd+1)
        
        % Display frame number being processed
        disp(['Processing frame ' num2str(k+startInd-chunkInds(1)) ' of ' num2str(numFrames-1) ' ...']);
        
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
        % R_body_to_world = rpy2rotmat([xCam(4);rpy0(2);xCam(6)]);
        camOrientation = R_body_to_world*xvec; % Orientation of x axis
        
        camPosition = xCam(1:3);
        % camPosition = camPosition + R_body_to_world*[0;-0.01;0];
        
        % "Up" vector of camera
        % camUp = [0;0;-1];
        camUp = R_body_to_world*[0;0;-1];
        
        % im_funnel = makeFunnelImage(funnel,funnelPos,camPosition,camOrientation,camUp,cameraParameters);
        im_funnel = changeView(figNum,camPosition,camOrientation,camUp,cameraParameters);
        
        
        % Superimpose funnel image onto image from video
        F = superImposeFunnel(im_undistorted,im_funnel,0.9);
        
        %     % Write frame to video using VideoWriter
        %     writeVideo(funnelVideo,F);
        
        % Write frame to video using avifile
        funnelVideo = addframe(funnelVideo,F);
        
        
    end
    
end

% close(funnelVideo);

% Close video object
funnelVideo = close(funnelVideo);

% figure
% imshow(im_funnel);

% ffmpeg -i funnelMovie_2015_02_20_03.avi -acodec libfaac -b:a 128k -vcodec mpeg4 -b:v 36000k -filter:v "setpts=0.25*PTS" -flags +aic+mv4 funnelMovie_2015_02_20_03.mp4






