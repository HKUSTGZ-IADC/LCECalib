function [TOptm,fval_out] = GlobalSearchOptm(edges_pts,corners3D,tf0)
%% 由于2D平面运动，这里优化的目标是旋转方向对应的旋转角度，以及旋转方向的平移，在此使用四元数进行优化
x0 = [0,0,0,0,0,0]; % 李代数　pha1,pha2,pha3,tx,ty,tz
TOptm = eye(4);
% step 1 find correspondent line

edge_pts_aft = tf0(1:3,1:3)*edges_pts+tf0(1:3,4);

lines_cor = findCorres(corners3D,edge_pts_aft);
%% 同时优化

% opts = optimoptions('fminunc','Display','iter','Algorithm','quasi-newton',...
%     'MaxFunctionEvaluations', 1000,'MaxIterations',1500,'UseParallel',true);
opts = optimoptions('fminunc','Algorithm','quasi-newton',...
    'MaxFunctionEvaluations', 1000,'MaxIterations',1500,'UseParallel',true);
FAll = @(x) object_func(x(1:3),x(4:6),tf0,edges_pts,corners3D,lines_cor);
% opts = optimoptions('fminunc','Algorithm','quasi-newton','Display','iter','MaxFunctionEvaluations', 10000,'MaxIterations',1500);
% [x,fval]=fminunc(FAll,x0,opts);
problem = createOptimProblem('fmincon','objective',FAll,'x0',x0, ...
    'lb',4*[-0.02,-0.02,-0.02,-0.1,-0.1,-0.1], ...
    'ub',4*[0.02,0.02,0.02,0.1,0.1,0.1],'options', ...
    opts);

% global search tutorial: https://www.mathworks.com/help/gads/how-globalsearch-and-multistart-work.html  
gs = GlobalSearch;
[x,f] = run(gs, problem);
R = eul2rotm(x(1:3));
TOptm(1:3,1:3) = R;
TOptm(1:3,4) = x(4:6)';
TOptm = TOptm*tf0;
fval_out=f;
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

function plotcorres(corner,pts_in,lines_cor)

line1_pts_idx = find(lines_cor==1);
line2_pts_idx = find(lines_cor==2);
line3_pts_idx = find(lines_cor==3);
line4_pts_idx = find(lines_cor==4);

pts1 = pts_in(:,line1_pts_idx);
pts2 = pts_in(:,line2_pts_idx);
pts3 = pts_in(:,line3_pts_idx);
pts4 = pts_in(:,line4_pts_idx);

figure;
axis equal;
plot3(corner(1,1:2),corner(2,1:2),corner(3,1:2),"o-r");
hold on;
plot3(pts1(1,:),pts1(2,:),pts1(3,:),".r");

plot3(corner(1,2:3),corner(2,2:3),corner(3,2:3),"o-g");
plot3(pts2(1,:),pts2(2,:),pts2(3,:),".g");
plot3(corner(1,3:4),corner(2,3:4),corner(3,3:4),"o-b");
plot3(pts3(1,:),pts3(2,:),pts3(3,:),".b");
plot3(corner(1,[4,1]),corner(2,[4,1]),corner(3,[4,1]),"o-black");
plot3(pts4(1,:),pts4(2,:),pts4(3,:),".black");
hold off;
end
