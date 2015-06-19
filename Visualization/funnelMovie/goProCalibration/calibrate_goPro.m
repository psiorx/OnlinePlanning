% Get images from calibration videos
N = 18; % Number of calibration pics

ims = {};
for k = 1:N
    disp(k);
    % Read video
    vid = VideoReader(['./calibrationVideos/1080_120_W/vid' num2str(k) '.MP4']);
    
    % Read image from video
    ims{k} = read(vid,10);
    clf;
    imshow(ims{k});
    
    % Write image to pic
    imwrite(ims{k},['./calibrationPics/1080_120_W/pic' num2str(k) '.tif']);
    
    
end

% Run calibrator
cameraCalibrator;
