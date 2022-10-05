function roi = computeLiDARROI(imageCorners3d, tolerance)
%helperComputeROI computes ROI in lidar coordinate system using
%   checkerboard 3d corners in camera coordinate system
%
% This is an example helper function that is subject to change or removal in
% future releases.

% Copyright 2019-2020 The MathWorks, Inc.

xCamera = imageCorners3d(1, :);
yCamera = imageCorners3d(2, :);
zCamera = imageCorners3d(3, :);

xLiDAR = zCamera;
yLiDAR = -xCamera;
zLiDAR = -yCamera;

xMaxLidar = max(xLiDAR) + tolerance;
xMinLidar = min(xLiDAR) - tolerance;

yMaxLidar = max(yLiDAR) + tolerance;
yMinLidar = min(yLiDAR) - tolerance;

zMaxLidar = max(zLiDAR) + tolerance;
zMinLidar = min(zLiDAR) - tolerance;

roi = [xMinLidar, xMaxLidar, yMinLidar, yMaxLidar, zMinLidar, zMaxLidar];
end