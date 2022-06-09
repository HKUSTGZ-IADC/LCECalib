clc, clear;
close all;

addpath("..");
addpath("../tools");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");

%%
params = load('real_data/real_data_4/params.mat');
borW = params.borW;
borH = params.borH;
pattern_size = params.pattern_size;
K = params.K;
D = params.D;
TGt = params.TGt;

cameraParams = load('real_data/real_data_4/calibration.mat');

%% detect frame camera features
img_frame = imread('real_data/real_data_4/img/00042.png');
imageSize = [size(img_frame,1),size(img_frame,2)];
im = undistortImage(img_frame, cameraParams.cameraParams);

[imagePoints,boardSize] = detectCheckerboardPoints(im);
worldPoints = generateCheckerboardPoints(boardSize, pattern_size);
worldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
[worldOrientation,worldLocation] = estimateWorldCameraPose(imagePoints,worldPoints,cameraParams.cameraParams);
T= [[worldOrientation;worldLocation],[0,0,0,1]'];
T = T';
T = inv(T);
worldPx=T(1:3,1:3) * worldPoints'+T(1:3,4);
worldPx = cameraParams.cameraParams.IntrinsicMatrix' * worldPx;
worldPx = worldPx(1:2,:)./worldPx(3,:);
imagePoints= imagePoints';
err = worldPx - imagePoints;
aver_err = sum(vecnorm(err))/size(err,2);

center = mean(worldPoints,1);
pts_corner = [[borW/2,borH/2,0]',[borW/2,-borH/2,0]',...
              [-borW/2,-borH/2,0]',[-borW/2,borH/2,0]'];
pts_corner= center' + pts_corner;
pts_corner = T(1:3,1:3)*pts_corner+T(1:3,4);
pc_corner_px= cameraParams.cameraParams.IntrinsicMatrix' * pts_corner;
pc_corner_px = pc_corner_px(1:2,:)./pc_corner_px(3,:);
pc_corner_px = [pc_corner_px, pc_corner_px(:, 1)];
plane_coeff = T(1:3,1:3)*[0,0,1]';
d = -plane_coeff'*T(1:3,4);
plane_coeff = [plane_coeff;d];
if d<0
    plane_coeff =-plane_coeff;
end

figure(1); imshow(im);
hold on;
plot(worldPx(1,:), worldPx(2,:),'.r', 'MarkerSize', 20);
plot(pc_corner_px(1,:), pc_corner_px(2,:), 'b', 'LineWidth', 3);
plot(pc_corner_px(1,:), pc_corner_px(2,:), '.g', 'MarkerSize', 30);
hold off;

hf = figure(1);
filename = 'real_data/real_data_4/mp_frame_image';
print(filename, '-depsc');  
saveas(hf, strcat(filename, '.fig'));  

%% detect event camera features
img_event = imread('real_data/real_data_5/img/00042.png');
imageSize = [size(img_event,1),size(img_event,2)];
im = undistortImage(img_event, cameraParams.cameraParams);

[imagePoints,boardSize] = detectCheckerboardPoints(im);
worldPoints = generateCheckerboardPoints(boardSize, pattern_size);
worldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
[worldOrientation,worldLocation] = estimateWorldCameraPose(imagePoints,worldPoints,cameraParams.cameraParams);
T= [[worldOrientation;worldLocation],[0,0,0,1]'];
T = T';
T = inv(T);
worldPx=T(1:3,1:3) * worldPoints'+T(1:3,4);
worldPx = cameraParams.cameraParams.IntrinsicMatrix' * worldPx;
worldPx = worldPx(1:2,:)./worldPx(3,:);
imagePoints= imagePoints';
err = worldPx - imagePoints;
aver_err = sum(vecnorm(err))/size(err,2);

center = mean(worldPoints,1);
pts_corner = [[borW/2,borH/2,0]',[borW/2,-borH/2,0]',...
              [-borW/2,-borH/2,0]',[-borW/2,borH/2,0]'];
pts_corner= center' + pts_corner;
pts_corner = T(1:3,1:3)*pts_corner+T(1:3,4);
pc_corner_px= cameraParams.cameraParams.IntrinsicMatrix' * pts_corner;
pc_corner_px = pc_corner_px(1:2,:)./pc_corner_px(3,:);
pc_corner_px = [pc_corner_px, pc_corner_px(:, 1)];
plane_coeff = T(1:3,1:3)*[0,0,1]';
d = -plane_coeff'*T(1:3,4);
plane_coeff = [plane_coeff;d];
if d<0
    plane_coeff =-plane_coeff;
end

figure(2); imshow(im);
hold on;
plot(worldPx(1,:), worldPx(2,:),'.r', 'MarkerSize', 20);
plot(pc_corner_px(1,:), pc_corner_px(2,:), 'b', 'LineWidth', 3);
plot(pc_corner_px(1,:), pc_corner_px(2,:), '.g', 'MarkerSize', 30);
hold off;

hf = figure(2);
filename = 'real_data/real_data_5/mp_event_image';
print(filename, '-depsc');  
saveas(hf, strcat(filename, '.fig'));  


%% detect LiDAR features
cloud = pcread('real_data/real_data_5/pcd/00042.pcd');
pc_array = cloud.Location()';
[pts_bor, bor_coeff, err] = boardpts_ext(pc_array, borW, borH);
pc_corners = borcorner_ext(pts_bor, borW, borH, 1);
pcshow(pts_bor', [1, 0, 0], 'MarkerSize', 30);

cloud_complete = pcread('real_data/real_data_5/pcd/00042_complete.pcd');
pcshow(cloud_complete.Location, [0.2, 0.2, 0.2], 'MarkerSize', 15);

xlabel("X [m]"); ylabel("Y [m]"); zlabel("Z [m]");
grid on;
ax = gca;
ax.GridLineStyle = '-';
ax.GridAlpha = 0.3;
set(gca, 'FontName', 'Times', 'FontSize', 40, 'LineWidth', 2);
axis([-1, 6, -1.6, 2.5, -0.9, 0.5]);

hf = figure(3);
filename = 'real_data/real_data_5/mp_cloud';
print(filename, '-depsc');  
saveas(hf, strcat(filename, '.fig'));  

%% plot specific point cloud
params = load('real_data/real_data_3/params.mat');
borW = params.borW;
borH = params.borH;
pattern_size = params.pattern_size;
K = params.K;
D = params.D;
TGt = params.TGt;
cameraParams = load('real_data/real_data_3/calibration.mat');

cloud = pcread('real_data/real_data_3/pcd/1638845914.249441.pcd');
pc_array = cloud.Location()';
[pts_bor, bor_coeff, err] = boardpts_ext(pc_array, borW, borH);
pc_corners = borcorner_ext(pts_bor, borW, borH, 1);

cloud_complete = pcread('real_data/real_data_3/raw_pcd/1638845914.249441.pcd');
pcshow(cloud_complete.Location, [0.2, 0.2, 0.2], 'MarkerSize', 15);
pcshow(pts_bor', [1, 0, 0], 'MarkerSize', 30);

xlabel("X [m]"); ylabel("Y [m]"); zlabel("Z [m]");
grid on;
ax = gca;
ax.GridLineStyle = '-';
ax.GridAlpha = 0.3;
set(gca, 'FontName', 'Times', 'FontSize', 40, 'LineWidth', 2);
% axis([-2, 6, -2, 2.5, -1.2, 0.5]);
axis([-0.5, 6, -3.5, 2, -1.2, 0.5]);
view(230, 20);

hf = figure(2);
filename = 'real_data/real_data_3/calibrate_cloud';
print(filename, '-depsc');  
saveas(hf, strcat(filename, '.fig'));  









