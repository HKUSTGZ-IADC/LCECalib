clc, clear;
close all;

%% image
img_raw = imread('/Titan/dataset/1_project_handheld_device/test/2022-02-05/data_calib/event_cam00/image/data/000015.png');
% img_raw = imread('/Titan/dataset/1_project_handheld_device/test/2022-02-15/data_calib_1/frame_cam00/data/000000.png');
imageSize = [size(img_raw,1), size(img_raw,2)];
figure; imshow(img_raw);

%% intrinsics 1 (MATLAB, 5 coefficients, jjiao)
K = [290.948 0 161.333; 
     0 291.0628 137.320;
     0 0 1];
D = [-4.08883393e-01, 3.19604844e-01, -1.79185998e-04, 2.19323905e-04, -2.51426339e-01];
IntrinsicMatrix = K';
radialDistortion = [D(1),D(2)];
tangentialDist = [D(3),D(4)];
principalPoint =[K(1,3),K(2,3)];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',...
                                radialDistortion,'TangentialDistortion',tangentialDist);
img_undist = undistortImage(img_raw, cameraParams);
figure; imshow(img_undist);

%% intrinsics 1 (MATLAB, xiang)
K = [299.43517 0 161.5057; 
     0 299.240402 129.04167;
     0 0 1];
D = [-0.38111 0.16379 0.002869 -0.000832265];
IntrinsicMatrix = K';
radialDistortion = [D(1),D(2)];
tangentialDist = [D(3),D(4)];
principalPoint =[K(1,3),K(2,3)];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',...
                                radialDistortion,'TangentialDistortion',tangentialDist);
img_undist = undistortImage(img_raw, cameraParams);
figure; imshow(img_undist);

%% intrinsics 2 (Kalibr, checkerboard, xiang)
K = [299.43517 0 161.5057;
     0 299.24040 129.0416;
     0 0 1];
D = [-0.38111209287724507 0.1637958933167437 0.002869320682413722 -0.0008322650773518868];
IntrinsicMatrix = K';
radialDistortion = [D(1),D(2)];
tangentialDist = [D(3),D(4)];
principalPoint =[K(1,3),K(2,3)];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',...
                              radialDistortion,'TangentialDistortion',tangentialDist);
img_undist = undistortImage(img_raw, cameraParams);
figure; imshow(img_undist);

%% intrinsics 3 (Kalibr, Apriltag, xiang)
K = [283.70139 0 161.66741;
     0 284.51998 161.6674;
     0 0 1];
D = [-0.34503769554791985 0.11871949117272915 -0.0004466859523717791 0.00018036381112371884];
IntrinsicMatrix = K';
radialDistortion = [D(1),D(2)];
tangentialDist = [D(3),D(4)];
principalPoint =[K(1,3),K(2,3)];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',...
                              radialDistortion,'TangentialDistortion',tangentialDist);
img_undist = undistortImage(img_raw, cameraParams);
figure; imshow(img_undist);







