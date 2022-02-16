clc, clear;
close all;

%% image
img_raw = imread('/Titan/dataset/1_project_handheld_device/test/2022-02-05/data_calib/frame_cam00/image/data/000015.png');
% img_raw = imread('/Titan/dataset/1_project_handheld_device/test/2022-02-15/data_calib_1/frame_cam00/data/000000.png');
imageSize = [size(img_raw,1), size(img_raw,2)];
figure; imshow(img_raw);

%% intrinsics 1 (MATLAB, 5 coefficients, jjiao)
K = [591.807 0 525.9599; 
     0 592.47448 400.94598;
     0 0 1];
D = [-1.04214795e-01, 1.11082807e-01, 1.37362024e-03, 1.96136790e-03, -2.49492340e-02];
IntrinsicMatrix = K';
radialDistortion = [D(1),D(2)];
tangentialDist = [D(3),D(4)];
principalPoint =[K(1,3),K(2,3)];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',...
                                radialDistortion,'TangentialDistortion',tangentialDist);
img_undist = undistortImage(img_raw, cameraParams);
figure; imshow(img_undist);

%% intrinsics 1 (MATLAB, 4 coefficients, jjiao)
K = [593.3            0       517.97
            0       594.47       401.59
            0            0            1];
D = [-0.093527     0.078816    0.0027358  -0.00051689];
IntrinsicMatrix = K';
radialDistortion = [D(1),D(2)];
tangentialDist = [D(3),D(4)];
principalPoint =[K(1,3),K(2,3)];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',...
                                radialDistortion,'TangentialDistortion',tangentialDist);
img_undist = undistortImage(img_raw, cameraParams);
figure; imshow(img_undist);

%% intrinsics 1 (MATLAB, xiang)
K = [605.1285854737644 0 521.4534042955955; 
     0 604.9740557965580 394.8784804702080;
     0 0 1];
D = [-0.091985129289602, 0.086698394032381, 0.0002486228038038945, 0.0007215657419448336];
IntrinsicMatrix = K';
radialDistortion = [D(1),D(2)];
tangentialDist = [D(3),D(4)];
principalPoint =[K(1,3),K(2,3)];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',...
                                radialDistortion,'TangentialDistortion',tangentialDist);
img_undist = undistortImage(img_raw, cameraParams);
figure; imshow(img_undist);

%% intrinsics 2 (Kalibr, checkerboard, xiang)
K = [599.183334483831 0 519.9705372656427;
     0 598.9906613843401 395.2290811975643;
     0 0 1];
D = [-0.08932314888320775 0.07850010818974336 0.0002912976611566907 0.0007266826633270884];
IntrinsicMatrix = K';
radialDistortion = [D(1),D(2)];
tangentialDist = [D(3),D(4)];
principalPoint =[K(1,3),K(2,3)];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',...
                              radialDistortion,'TangentialDistortion',tangentialDist);
img_undist = undistortImage(img_raw, cameraParams);
figure; imshow(img_undist);

%% intrinsics 3 (Kalibr, Apriltag, xiang)
K = [599.4965129905646, 0 519.1561103859508;
     0 601.777510739006 394.8561003413857;
     0 0 1];
D = [-0.08769948998679322 0.07751744996850239 0.00018762052625616006 0.0007135227165142899];
IntrinsicMatrix = K';
radialDistortion = [D(1),D(2)];
tangentialDist = [D(3),D(4)];
principalPoint =[K(1,3),K(2,3)];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',...
                              radialDistortion,'TangentialDistortion',tangentialDist);
img_undist = undistortImage(img_raw, cameraParams);
figure; imshow(img_undist);







