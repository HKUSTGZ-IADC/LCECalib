function [pts_corner,plane_coeff,aver_err] = ...
  imgbor_ext(img,K,D,pattern_size,borW,borH,isdisplay)

IntrinsicMatrix = K';
if (length(D) == 4)
  radialDistortion = [D(1),D(2)];
else
  radialDistortion = [D(1),D(2),D(5)];
end
tangentialDist = [D(3),D(4)];
principalPoint =[K(1,3),K(2,3)];
cam_param = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',...
                              radialDistortion,'TangentialDistortion',tangentialDist);
imageSize = [size(img,1),size(img,2)];
im = undistortImage(img,cam_param);

%% detect checkerboard points and estimate the board pose
[imagePoints,boardSize] = detectCheckerboardPoints(im);
worldPoints = generateCheckerboardPoints(boardSize, pattern_size);
worldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
[worldOrientation,worldLocation] = estimateWorldCameraPose(imagePoints,worldPoints,cam_param);
T= [[worldOrientation;worldLocation],[0,0,0,1]'];
T = T';
T = inv(T);
worldPx=T(1:3,1:3)*worldPoints'+T(1:3,4);
worldPx = K*worldPx;
worldPx = worldPx(1:2,:)./worldPx(3,:);
imagePoints= imagePoints';
err = worldPx - imagePoints;
aver_err = sum(vecnorm(err))/size(err,2);

%% estimate checkerboard corners
center = mean(worldPoints,1);
pts_corner = [[borW/2,borH/2,0]',[borW/2,-borH/2,0]',...
              [-borW/2,-borH/2,0]',[-borW/2,borH/2,0]'];
pts_corner= center' + pts_corner;
pts_corner = T(1:3,1:3)*pts_corner+T(1:3,4);
pc_corner_px= K*pts_corner;
pc_corner_px = pc_corner_px(1:2,:)./pc_corner_px(3,:);
plane_coeff = T(1:3,1:3)*[0,0,1]';
d = -plane_coeff'*T(1:3,4);
plane_coeff = [plane_coeff;d];
if d<0
    plane_coeff =-plane_coeff;
end

if isdisplay
  figure;
  imshow(im);
  hold on;
  hold on;
  plot(worldPx(1,:),worldPx(2,:),'.r');
  plot(pc_corner_px(1,:),pc_corner_px(2,:),'-ob');
  hold off;
end
end