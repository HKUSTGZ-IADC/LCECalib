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

xMaxLidar = max(zCamera) + tolerance;
xMinLidar = min(zCamera) - tolerance;

yMaxLidar = max(xCamera) + tolerance;
yMinLidar = min(xCamera) - tolerance;

zMaxLidar = max(yCamera) + tolerance;
zMinLidar = min(yCamera) - tolerance;

roi = [xMinLidar, xMaxLidar, yMinLidar, yMaxLidar, zMinLidar, zMaxLidar];
end