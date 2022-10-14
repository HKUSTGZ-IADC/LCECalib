function Tout = corner_optm(cam_corners,pc_corners,tf0)

%% step 1 find correspondence
pts_ref = [];
pts_mov = [];
for idx = 1:size(cam_corners,2)
    cur_cam_corner = cam_corners{idx};
    cur_pc_corner = pc_corners{idx};
    cur_pc_corner_aft = tf0(1:3,1:3)*cur_pc_corner+tf0(1:3,4);
    for idxidx=1:size(cur_pc_corner_aft,2)
        min_dis = 10;
        min_idx= 0;
       for cam_idx = 1:size(cur_cam_corner,2)
          dis = cur_pc_corner_aft(:,idxidx) - cur_cam_corner(:,cam_idx);
          dis = norm(dis);
          if min_dis>dis
            min_dis = dis;
            min_idx = cam_idx;
          end
       end
       if min_dis < 10
          pts_ref = [pts_ref,cur_cam_corner(:,min_idx)];
          pts_mov = [pts_mov,cur_pc_corner(:,idxidx)];
       end
    end
end

% pts_mov_aft = tf0(1:3,1:3)*pts_mov+tf0(1:3,4);

% figure;
% axis equal;
% plot3(pts_mov_aft(1,:),pts_mov_aft(2,:),pts_mov_aft(3,:),"or");
% hold on;
% plot3(pts_ref(1,:),pts_ref(2,:),pts_ref(3,:),"*b");
Tout = TransformationSVDSolver(pts_ref, pts_mov);
%% use svd to solve this problem


end