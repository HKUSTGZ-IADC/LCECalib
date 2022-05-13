function [img_undist, camParams] = undistort_image(img, K, D)
  IntrinsicMatrix = K';
  if (length(D) == 4)
    radialDistortion = [D(1),D(2)];
  else
    radialDistortion = [D(1),D(2),D(5)];
  end
  tangentialDist = [D(3), D(4)];
  principalPoint =[K(1,3), K(2,3)];
  camParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,...
    'RadialDistortion', radialDistortion,...
    'TangentialDistortion',tangentialDist);
  imageSize = [size(img,1),size(img,2)];
  img_undist = undistortImage(img, camParams);
end