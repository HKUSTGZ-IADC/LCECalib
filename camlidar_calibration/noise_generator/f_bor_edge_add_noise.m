function pts_out = f_bor_edge_add_noise(R,t,pts_edge2,diverg_angle,bor_w,bor_h)
bor_corners = [bor_h/2,bor_w/2,0;...
               -bor_h/2,bor_w/2,0;...
               -bor_h/2,-bor_w/2,0;...
               bor_h/2,-bor_w/2,0;...
               bor_h/2,bor_w/2,0]';
pts_out=[];
bor_corners_aft = R*bor_corners+t;

p_n = R*[0,0,1]';
p_d = -t'*R*[0,0,1]';
for idx=1:size(pts_edge2,2)
    pt = pts_edge2(:,idx);
    dir = pt./norm(pt);
    pt_on_bor = -(p_d/(p_n'*dir))*dir;
    min_dis=10;
    min_idx=0;
    for idx2=1:4
        bor_edge = bor_corners_aft(:,idx2+1) - bor_corners_aft(:,idx2);
        bor_edge_dir = bor_edge./norm(bor_edge);
        corn2pt = pt_on_bor - bor_corners_aft(:,idx2);
        v_dis = corn2pt'*bor_edge_dir*bor_edge_dir-corn2pt;
        if norm(v_dis)<min_dis
            min_dis = norm(v_dis);
            min_idx = idx2;
        end
    end
    bor_edge = bor_corners_aft(:,min_idx+1) - bor_corners_aft(:,min_idx);
    bor_edge_dir = bor_edge./norm(bor_edge);
    corn2pt = pt_on_bor - bor_corners_aft(:,min_idx);
    pt_pro = corn2pt'*bor_edge_dir*bor_edge_dir+bor_corners_aft(:,min_idx);
    cos_theta = pt_pro'*dir./norm(pt_pro);
    if cos(diverg_angle/2)<cos_theta
        pts_out = [pts_out,pt_on_bor];
    else
        pts_out=[pts_out,pt];
    end
end
end