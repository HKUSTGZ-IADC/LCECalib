clc, clear; 
close all;

%%
img_filename = "/home/jjiao/Docker_ws/docker_fold/documents/dataset/event_camera/e2calib/davis346/LE_calib_20211228-RI-1/e2calib/00047.png";
I = imread(img_filename);
[imagePoints, boardSize] = detectCheckerboardPoints(I);
J = insertMarker(I, imagePoints,'o','Color','red','Size',2);
imshow(J);
