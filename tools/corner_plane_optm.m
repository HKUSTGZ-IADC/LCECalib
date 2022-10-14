function Tout = corner_plane_optm(cam_corners,cam_bor_coeff,pc_bor_corners,pc_bors_pts,tf0)

%% step 1 find correspondence
pts_ref = [];
pts_mov = [];
for idx = 1:size(cam_corners,2)
    cur_cam_corner = cam_corners{idx};
    cur_pc_corner = pc_bor_corners{idx};
    cur_pc_corner_aft = tf0(1:3,1:3)*cur_pc_corner+tf0(1:3,4);
    for idxidx=1:size(cur_pc_corner_aft,2)
        min_dis = 10;
        min_idx= 0;
       for cam_idx = 1:size(cur_cam_corner,2)
          dis = cur_pc_corner_aft(:,idxidx)- cur_cam_corner(:,cam_idx);
          dis = norm(dis);
          if min_dis>dis
            min_dis = dis;
            min_idx = cam_idx;
          end
       end
       if min_dis<10
          pts_ref = [pts_ref,cur_cam_corner(:,min_idx)];
          pts_mov = [pts_mov,cur_pc_corner(:,idxidx)];
       end
    end
end

%% use svd to solve this problem
Tout = CornerPlaneSolver(pts_ref,cam_bor_coeff,pts_mov,pc_bors_pts,tf0);
end

function Tout=CornerPlaneSolver(cam_corners,cam_bors_coeff,pc_bors_corners,pc_bors_pts,tf0)

F = @(x) obj_func(x(1:3),x(4:6),cam_corners,cam_bors_coeff,pc_bors_corners,pc_bors_pts,tf0);
x0 = [0,0,0,0,0,0];
opts = optimoptions('fminunc','Algorithm','quasi-newton','Display','final','MaxFunctionEvaluations', 10000,'MaxIterations',1500);
[x_optm,fval]=fminunc(F,x0,opts);
delatR = eul2rotm(x_optm(1:3));
% delatR = delatR.R;
delatT = [[delatR,x_optm(4:6)'];[0,0,0,1]];
Tout = delatT*tf0;
end


function error =obj_func(rvec,tvec,cam_corners,cam_bors_coeff,pc_bors_corners,pc_bors_pts,tf0)

error=double(0);
R = eul2rotm(rvec);

%% corner loss

pc_bor_corners_aft = tf0(1:3,1:3)*pc_bors_corners+tf0(1:3,4);
pc_bor_corners_aft = R*pc_bor_corners_aft + tvec';

corner_loss = cam_corners - pc_bor_corners_aft;
corner_loss = vecnorm(corner_loss);
error1 = sum(corner_loss,'all');

%% point-to-plane loss
error2=0;
for idx=1:size(cam_bors_coeff,1)
   bor_coeff = cam_bors_coeff{idx};
   bor_pts = pc_bors_pts{idx};
   bor_pts_aft = tf0(1:3,1:3)*bor_pts +tf0(1:3,4);
   bor_pts_aft = R*bor_pts_aft+tvec';
   dis = bor_coeff(1:3)'*bor_pts_aft+bor_coeff(4);
   dis = abs(dis);
   error2 =error2+ sum(dis,'all')/size(dis,2);
end
    
error = error1/size(corner_loss,2)+error2/size(cam_bors_coeff,1);

error =double(error);
end




