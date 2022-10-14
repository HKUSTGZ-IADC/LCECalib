function [model,inlinerIdx] = plane_ransac(pts_in,max_dis)
  sampleSize = 5;
  maxDistance = max_dis;
  fitFcn = @(pts) PlaneFitIn3D(pts);
  distFcn = @(model,pts) Plane3dVal(model,pts);
  [model,inlinerIdx] = ransac(pts_in',fitFcn,distFcn,sampleSize,maxDistance);
  model = pts_in(:,inlinerIdx)'\(-ones(size(pts_in(:,inlinerIdx),2),1));
  model = [model;1];
  model = model/norm(model(1:3));
end
