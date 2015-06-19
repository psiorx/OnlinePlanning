close all;
figure(1); hold on;
% Hoop
% r = 0.44; % radius of hoop in m
rh = 44.5/100; % parameter that controls size of hoop

w = 0.01; % Thickness of hoop divided by two
l = 2*rh*tan(pi/8); % length of each segment of approximating hexagon
d = l/2;

% First segment (top most)
verts = [-w w w -w -w w w -w;-d -d d d -d -d d d;w w w w -w -w -w -w];
verts1 = verts + repmat([0;0;-rh],1,size(verts,2));
hoop{1} = verts1;

P1 = polytope(verts1');
plot(P1);

% Second segment (bottom most)
verts2 = verts + repmat([0;0;rh],1,size(verts,2));
hoop{2} = verts2;

P2 = polytope(verts2');
plot(P2);

% Third segment (left most)
R = rpy2rotmat([pi/2;0;0]);
verts3 = R*verts + repmat([0;-rh;0],1,size(verts,2));
hoop{3} = verts3;

P3 = polytope(verts3');
plot(P3);

% Fourth segment (right most)
verts4 = R*verts + repmat([0;rh;0],1,size(verts,2));
hoop{4} = verts4;

P4 = polytope(verts4');
plot(P4);

% Fifth segment (between 1 and 3)
verts5 = [verts1(:,[1,2,5,6]),verts3(:,[1,2,5,6])];
hoop{5} = verts5;

P5 = polytope(verts5');
plot(P5);

% Sixth segment (between 1 and 4)
verts6 = [verts1(:,[3,4,7,8]),verts4(:,[1,2,5,6])];
hoop{6} = verts6;

P6 = polytope(verts6');
plot(P6)

% Seventh segment (between 2 and 3)
verts7 = [verts2(:,[1,2,5,6]),verts3(:,[3,4,7,8])];
hoop{7} = verts7;

P7 = polytope(verts7');
plot(P7)

% Eigth segment (between 2 and 4)
verts8 = [verts2(:,[3,4,7,8]),verts4(:,[3,4,7,8])];
hoop{8} = verts8;

P8 = polytope(verts8');
plot(P8)





% REVERSE DIRECTION OF PLOTTING (to match coordinate frame)
set(gca,'YDir','reverse')
set(gca,'ZDir','reverse')
axis([-1,1,-1,1,-1,1]);
