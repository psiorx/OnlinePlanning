close all;
% Load funnel
% load('../../FunnelLibrary/funnelLibrary_Feb10_funnel.mat');
load('./funnelSphere.mat');

% Plot funnel
% figure(1);
im_sz = [1920, 1080];
figure('Position',[100 100 im_sz]);
options = struct();
options.inclusion = 'projection';
% options.ts = funnelLibrary(1).ts;
V = funnelLibrary(1).V;
% options.x0 = funnelLibrary(1).xtraj;
options.plotdims = [1 2 3];
options.num_samples = 1000;

% plot3(grid_org(1),grid_org(2),grid_org(3),'ro');
% plotShiftedFunnel(V*(1000/100)^2,[69/1000,69/1000,28.19/1000]',options);
plotShiftedFunnel(V*(1000/100)^2,grid_org,options);
% plotShiftedFunnel(V*1000,[grid_org;zeros(9,1)],options);


% REVERSE DIRECTION OF PLOTTING (to match coordinate frame)
set(gca,'YDir','reverse')
set(gca,'ZDir','reverse')
axis equal;

% Camera properties
camera_pos = [0 0 0];
camera_target = [1 0 0] + camera_pos;
% fov = 2*atand(im_sz(1)/f);

% Get focal length from intrinsic matrix of camera
focal = KK(1,1); % mid/tand(fov/2);
% Define field of view of the camera in degrees
% fov = 57.73;
midx = im_sz(1)/2;
midy = im_sz(2)/2;
fov = 2*atand(midy/focal);
camva(fov);  % Set the camera field of view
% view(0,0)
camva(fov);
campos(camera_pos);
camtarget(camera_target);
camproj('perspective');

% Now set axis properties
axis image
axis off
set(gca,'Units','pixels');
set(gca,'Position',[1 1 im_sz]);
set(gcf, 'Color', [1 1 1])

% im_funnel = frame2im(getframe(gcf));
F = getframe(gcf);
im_funnel = frame2im(F); %  rgb2gray(F.cdata);

imshow(im_funnel,[]), impixelinfo;




