close all;

% % File extension (corresponds to lcm logs)
% file_num = '2015_02_24_17';
% 
% % Read video
% vid = VideoReader(['./rawFunnelMovies/video_' file_num '.MP4']);
% 
% % Undistort using gopro calibration parameters
% load('./goProCalibration/cameraParams_1080_120_W.mat');
% 
% startFrame = 1;
% 
% % Now get camera positions
% [xtrajCam,xtrajSBach,q_cross] = getCameraTraj(['../../../Logs/Feb242015/lcmlog_' file_num '.mat']);
% % return;
% 
% % Load funnel
% load('../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');
% % load('./funnelSphere.mat');
% 
% funnel = funnelLibrary(1);
% 
% % X-vector of camera
% % xvec = [1;-0.03;-0.02];
% xvec = [1;0;0];
% xvec = xvec/norm(xvec);
% 
% % Number of frames in video
% N = vid.NumberOfFrames;
% 
% % Camera orientation
% % Starting position of funnel
% % funnelPos = [-0.6;0;-1.54;zeros(9,1)]; % -1.43
% funnelPos = [q_cross(1:3);zeros(9,1)];
% 
% % Get position of camera
% tCam = xtrajCam.getBreaks();
% xsCam = xtrajCam.eval(tCam);
% for k = 1:6
%     figure
%     plot(xsCam(k,:));
%     % xlim([0 tCam(end)]);
% end
% 
% t_start_cam = input('Input camera starting index:');
% t_end_cam = input('Input camera ending index:');
% 
% close all;

xCam = mean(xsCam(:,t_start_cam:t_end_cam),2);

% Camera orientation
% camOrientation = [1;0;0];
R_body_to_world = rpy2rotmat(xCam(4:6));
% R_body_to_world = rpy2rotmat([xCam(4);rpy0(2);xCam(6)]);
camOrientation = R_body_to_world*xvec; % Orientation of x axis

camPosition = xCam(1:3);
% camPosition = camPosition + R_body_to_world*[0;-0.01;0];

% "Up" vector of camera
camUp = R_body_to_world*[0;0;-1];

% Plot funnel (just once since camera is not
% moving)
[im_funnel,figNum] = makeFunnelImage(funnel,funnelPos,camPosition,camOrientation,camUp,cameraParameters);

% return;

funnelVideo = avifile(['./funnelMovies/funnelMovie_' file_num  '.avi']);
funnelVideo.Quality = 100;
funnelVideo.Compression = 'None';

fps = vid.FrameRate;
funnelVideo.Fps = fps;

% Read all images from video
startTime = 38; % 47
endTime = 41; % 50; 
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
        
        % Load frame as image
        % im = read(vid,k);
        im = ims(:,:,:,k);
        
        % Undistort images using go pro camera params
        im_undistorted = undistortImage(im, cameraParameters);
        
        
        % Superimpose funnel image onto image from video
        F = superImposeFunnel(im_undistorted,im_funnel,0.7); % 0.65
        
        
        % Write frame to video using avifile
        funnelVideo = addframe(funnelVideo,F);
        
        
    end
    
end

% Close video object
funnelVideo = close(funnelVideo);

% ffmpeg -i funnelMovie_2015_02_20_03.avi -acodec libfaac -b:a 128k -vcodec mpeg4 -b:v 72000k -filter:v "setpts=0.25*PTS" -flags +aic+mv4 funnelMovie_2015_02_20_03.mp4

% Process for making videos after this:
% - Get this into openshot along with original raw video
% - Take audio from raw video
% - Make sure to have an extra second of audio in the beginning. This is
% because openshot creates a blip in the audio right at the beginning.
% Trim extra second using ffmpeg:
% ffmpeg -ss 00:00:01 -t 00:00:03 -i final_movie_2015_02_24_00_extra.mp4 -acodec copy -vcodec copy final_movie_2015_02_24_00.mp4







