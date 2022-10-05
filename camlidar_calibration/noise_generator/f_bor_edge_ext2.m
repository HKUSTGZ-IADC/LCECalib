function [pts_edge,pts_edge2] = f_bor_edge_ext2(pts_in,borW,borH)
pts_edge=[];
pts_edge2=[];
[borpts,plane_model,~] = boardpts_ext(pts_in,borW,borH);
edge_pts_idx = find_pts_ring_edges(borpts);
edge_pts = borpts(:,edge_pts_idx);

thetaAlpha = [[],[]];
for index = 1: size(pts_in,2)
    R = norm(pts_in(:,index));
    z = pts_in(3,index);
    theta = round(asind(z/R));
    alpha = atan2d(pts_in(2,index),pts_in(1,index))+180;
    
    thetaAlpha = [thetaAlpha,[theta;alpha]];
end
pts_in_theta = [pts_in;thetaAlpha];
num=3;
for idx=1:size(edge_pts,2)
    pt = edge_pts(:,idx);
    R = norm(pt);
    z = pt(3);
    theta = round(asind(z/R));
    alpha = atan2d(pt(2),pt(1))+180;
    thetaidx = find(pts_in_theta(4,:)==theta);
    same_ring_pts = pts_in_theta(:,thetaidx);
    same_ring_pts = sort(same_ring_pts,5);
    dis = same_ring_pts(1:3,:) - pt;
    dis = vecnorm(dis);
    [V,I]=min(dis);
    for idx2=1:num
        I2 = I-idx2;
        if I2<=0
            I2 = size(same_ring_pts,2);
        end
        I3 = I+idx2;
        if I3>size(same_ring_pts,2)
            I3=1;
        end
        pt1 = same_ring_pts(1:3,I2);
        pt2 = same_ring_pts(1:3,I3);
        dis1 = plane_model(1:3)'*pt1+plane_model(4);
        dis2 = plane_model(1:3)'*pt2+plane_model(4);
        if abs(dis1)>0.01
            if abs(alpha-same_ring_pts(5,I2))<0.3*num
                pts_edge2 = [pts_edge2,pt1];
            end
        end
        if abs(dis2)>0.01
            if abs(alpha-same_ring_pts(5,I3))<0.3*num
                pts_edge2 = [pts_edge2,pt2];
            end
        end
    end
end
pts_edge = edge_pts;
end