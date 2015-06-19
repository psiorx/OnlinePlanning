close all;
% Read image from video
% vid = VideoReader('./testPics/checker1.MP4');
% im = read(vid,10);

% % CHANGE grid_org INDEX IF YOU CHANGE THIS
im = imread('./goProCalibration/calibrationPics/1080_120_W/pic17.tif');

% Undistort using gopro calibration parameters
load('./goProCalibration/cameraParams_1080_120_W.mat');
im_undistorted = undistortImage(im, cameraParameters); 

% Intrinsic matrix
KK = cameraParameters.IntrinsicMatrix;

% Figure out coordinate of origin of grid using extrinsics
grid_org = cameraParameters.TranslationVectors(9,:)';
grid_org = grid_org/1000; % convert to m
grid_org = [grid_org(3); grid_org(1); grid_org(2)]; % bring to my coordinate frame

close all;
% Make 3d plot of a sphere
plotSphereImage; 
close all;

% superimpose funnel plot on image
superImposeFunnel(im_undistorted,im_funnel,0.7);














