function Tout = lines_optm(edges_pts,corners3D,tf0)
% tf0: transformation matrix
% K: camera intrinsic
% edges_pts: lidar points, type : cell, for each cell dimenstion is  dim*npts
% corners3D: 相机坐标系下标定板的3D角点


is_display = 0;
corres_cam_lines = {};
% step 1 find correspondent line
edge_pts_aft = tf0(1:3,1:3)*edges_pts+tf0(1:3,4);

line_cor = findCorres(corners3D,edge_pts_aft);
corres_cam_lines = line_cor;



%% step 2 optimize

F = @(x) object_func(x(1:3),x(4:6),tf0,edges_pts,corners3D,corres_cam_lines);
x0 = [0,0,0,0,0,0];
opts = optimoptions('fminunc','Algorithm','quasi-newton','Display','iter','MaxFunctionEvaluations', 10000,'MaxIterations',1500);
[x_optm,fval]=fminunc(F,x0,opts);
delatR = eul2rotm(x_optm(1:3));
% delatR = delatR.R;
delatT = [[delatR,x_optm(4:6)'];[0,0,0,1]];
Tout = delatT*tf0;

%% optimize translation
% F = @(x) object_trans(x(1:3),tf0,edges_pts,corners3D,corres_cam_lines);
% x0 = [0,0,0];
% opts = optimoptions('fminunc','Algorithm','quasi-newton','MaxFunctionEvaluations', 10000,'MaxIterations',1500);
% [x_optm,fval]=fminunc(F,x0,opts);
% delatT = [[eye(3),x_optm(1:3)'];[0,0,0,1]];
% Tout = delatT*tf0;
end

function error = object_func(rvec,tvec,TInit,edges_pts,corners,lines_cor)
R = eul2rotm(rvec);
% R = R.R;
error=0;
edge_pts_aft = TInit(1:3,1:3)*edges_pts + TInit(1:3,4);
edge_pts_aft_aft = R*edge_pts_aft+tvec';
lines_cor = findCorres(corners,edge_pts_aft_aft);
for idxidx=1:size(lines_cor,2)
    line_idx = lines_cor(idxidx);
    pt1 = corners(:,line_idx);
    if line_idx==4
        pt2 = corners(:,1);
    else
        pt2 = corners(:,line_idx+1);
    end
    line_dir = pt2-pt1;
    line_dir = line_dir./norm(line_dir);
    dis = (eye(3) - line_dir*line_dir')*(edge_pts_aft_aft(:,idxidx) - pt1);
    error = error + double(norm(dis)*norm(dis));
end

end


function line_cor = findCorres(corners,pts_in)
dir = zeros(3,4);
dir(:,1) = corners(:,2) - corners(:,1);
dir(:,2)= corners(:,3) - corners(:,2);
dir(:,3) = corners(:,4) - corners(:,3);
dir(:,4) = corners(:,1) - corners(:,4);

dir(:,1)= dir(:,1)./norm(dir(:,1));
dir(:,2)= dir(:,2)./norm(dir(:,2));
dir(:,3)= dir(:,3)./norm(dir(:,3));
dir(:,4)= dir(:,4)./norm(dir(:,4));
line_cor=[];
for idx=1:size(pts_in,2)
    min_dis=10;
    min_idxidx=0;
    for idxidx=1:4
        dis = (eye(3) - dir(:,idxidx)*dir(:,idxidx)')*(pts_in(:,idx)-corners(:,idxidx));
        dis = norm(dis);
        if dis<min_dis
            min_dis = dis;
            min_idxidx = idxidx;
        end
    end
    line_cor = [line_cor,min_idxidx];
end
end

