function TOut = bor_icp(pts_target,pts_source)
% pts_target: target pointcloud
% pts_source: source cloud, here we suppose source cloud is on the x-o-y
% plane
% Tout: the transformation from source cloud to target cloud

% init transformation from board to lidar for icp is easy to fall into
% local solution
pts_target_normal = -pinv(pts_target')*ones(size(pts_target,2),1);
pts_target_normal = pts_target_normal./norm(pts_target_normal);
source_normal = [0,0,1]';

rot_vec = cross(source_normal,pts_target_normal);
theta = acos(pts_target_normal'*source_normal);
R_init = axang2rotm([rot_vec',theta]);;
T_init = sum(pts_target,2)./size(pts_target,2) - R_init*sum(pts_source,2)./size(pts_source,2);
Tf0=eye(4);
Tf0(1:3,1:3) = R_init;
Tf0(1:3,4) = T_init;
Kernel = 'L2';
transform= f_irls_icp(pts_target,pts_source, Tf0, Kernel);

%% for a rectangle board, icp will converge to local solution, 
% so we are about to generate some candidate soultions to re-estimate the transformation 
theta = [0:5:90];
q = [];
for idx = 1:size(theta,2)
   q = [q;[cosd(theta(idx)/2),sind(theta(idx)/2)*source_normal']];
end
min_error=10;
TOut = eye(4);
for idx=1:size(theta,2)
   TOut(1:3,1:3) = transform(1:3,1:3)*quat2rotm(q(idx,:));
   [RR,TT,tmp,error] = f_irls_icp(pts_target, pts_source, TOut, Kernel);
   if error<min_error
       min_error = error;
       TOut(1:3,1:3) = RR;
       TOut(1:3,4) = TT;
   end
end
TOut = transform;
% figure;
% axis equal;
% plot3(pts_target(1,:),pts_target(2,:),pts_target(3,:),'.r');
% hold on;
% pts_source_aft = TOut(1:3,1:3)*pts_source+TOut(1:3,4);
% plot3(pts_source_aft(1,:),pts_source_aft(2,:),pts_source_aft(3,:),'.b');

end