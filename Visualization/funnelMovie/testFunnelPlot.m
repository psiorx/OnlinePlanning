% Read image from video
% vid = VideoReader('./testPics/lab_4_65_1.MP4');
% im = read(vid,10); 
im = imread('./testPics/labPic1.tif');

% Undistort using gopro calibration parameters
load('./goProCalibration/cameraParams_1080_120_N.mat');
im_undistorted = undistortImage(im, cameraParameters); 

% Make 3d funnel plot
createNormalizedFunnelImage;

% superimpose funnel plot on image
h = superImposeFunnel(im_undistorted,im_funnel,0.7);
