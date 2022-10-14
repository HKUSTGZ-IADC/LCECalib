function [model,inlierIdx]= line3D_ransac(pts_in)
  % Fit a line using linear least squares.
  sampleSize = 2;
  maxDistance = 0.025;
  fitFcn = @(pts) fitFunc(pts);
  distFcn = @(model,pts) distFunc(model,pts);
   [model,inlierIdx] = ransac(pts_in',fitFcn,distFcn,sampleSize,maxDistance);
end

% 输入两个点确定一条直线，直线方程由一个点，和方向构成　[pt, dir]
function model = fitFunc(pt)
dir = pt(2,:) - pt(1,:);
dir = dir./norm(dir);
model = [pt(1,:),dir];
end

function dis = distFunc(model,data)
  dis = [];
  for idx = 1:size(data,1)
    sub_dis = (model(4:6)'* model(4:6) - eye(3))*(data(idx,:)'-model(1:3)');
    dis = [dis;norm(sub_dis)];
  end
end