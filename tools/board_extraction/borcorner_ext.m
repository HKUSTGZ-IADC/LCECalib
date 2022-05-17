function [corners, edge_pts] = borcorner_ext(bor_pts,borW,borH,is_display)
corners=[];

%% 先用icp找到一个候选解
sample_rate=200;
x = 0:borW/sample_rate:borW;
y = 0:borH/sample_rate:borH;

sample_pts=[];
sample_pts =[sample_pts,[x;zeros(2,size(x,2))]];
sample_pts =[sample_pts,[x;borH*ones(1,size(x,2));zeros(1,size(x,2))]];
sample_pts =[sample_pts,[zeros(1,size(y,2)-2);y(2:end-1);zeros(1,size(y,2)-2)]];
sample_pts =[sample_pts,[borW*ones(1,size(y,2)-2);y(2:end-1);zeros(1,size(y,2)-2)]];
sample_pts = sample_pts - [borW/2,borH/2,0]';

Tf0 = eye(4);
[plane_n,inlierIdx] = plane_ransac(bor_pts(1:3, :), 0.02);
plane_n = plane_n(1:3);
ax = -cross(plane_n,[0,0,1]');
angle = [0,0,1]*plane_n;
angle = acosd(angle);
Tf0(1:3,1:3) = axang2rotm([ax',angle*pi/180]);
Tf0(1:3,4) = mean(bor_pts(1:3, :), 2);
edge_pts_idx = find_pts_ring_edges(bor_pts);
edge_pts = bor_pts(:, edge_pts_idx);

%% 由于上面的icp会算出很多局部最有解，所以在这里我们旋转现在的解得到多个解，选其中损失最小的
theta = [0:5:90];
q = [];
source_normal =[0,0,1]';
for idx = 1:size(theta,2)
    q = [q;[cosd(theta(idx)),sind(theta(idx))*source_normal']];
end
min_error=10;
Ttmp= Tf0;
TInit = Tf0;
for idx=1:size(theta,2)
    deltaT = [[quat2rotm(q(idx,:)),[0,0,0]'];[0,0,0,1]];
    Ttmp = inv(deltaT*inv(Tf0));
    
    sample_pts_tmp = Ttmp(1:3,1:3)*sample_pts+Ttmp(1:3,4);
    error = suqare_dis(edge_pts(1:3, :), sample_pts_tmp);
    if error<min_error
        min_error = error;
        TInit=Ttmp;
    end
end
plane_coeff = plane_ransac(bor_pts(1:3, :),0.03);
qn = plane_coeff(1:3);
pts_onboard=[];
for idx=1:size(bor_pts, 2)
    pt = bor_pts(1:3, idx);
    ptonboard =[0,0,-plane_coeff(4)/plane_coeff(3)]';
    vec = pt - ptonboard;
    a = qn'*vec*qn;
    pts_onboard = [pts_onboard, [vec - a + ptonboard; bor_pts(4, idx)]];
end
edge_pts_idx = find_pts_ring_edges(pts_onboard);
edge_pts = pts_onboard(:, edge_pts_idx);

%% 使用最小二乘的方式，优化外参，使得边界点
corner3D = [[0;0;0],[borW;0;0],[borW;borH;0],[0;borH;0]];
corner3D = corner3D - [borW/2;borH/2;0];
TInit = inv(TInit);
TOptm = GlobalSearchOptm(edge_pts(1:3, :), corner3D,TInit);
TOptm = GlobalSearchOptm(edge_pts(1:3, :), corner3D,TOptm);
TOptm = inv(TOptm);
corners = TOptm(1:3,1:3)*corner3D+TOptm(1:3,4);

if is_display
    figure
    axis equal
    plot3(edge_pts(1,:),edge_pts(2,:),edge_pts(3,:),'.b', 'MarkerSize', 30);
    hold on
    sample_pts2 = TInit(1:3,1:3)*sample_pts+TInit(1:3,4);
    plot3([corners(1,:),corners(1,1)],[corners(2,:),corners(2,1)],[corners(3,:),corners(3,1)],'-b', 'LineWidth', 4);
    plot3([corners(1,:),corners(1,1)],[corners(2,:),corners(2,1)],[corners(3,:),corners(3,1)],'.g', 'MarkerSize', 50);
end

end

function [dis]=suqare_dis(pts_target,pts_source)

ns = createns(pts_source','nsmethod','kdtree');
[idx,dist]= knnsearch(ns,pts_target','k',1);
dis = sum(abs(dist));
end

