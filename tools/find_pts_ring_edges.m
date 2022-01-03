function idx_out = find_pts_ring_edges(pts_in)
% pts_in dim*npts
% find edge points for 16 lidar
idx_out =[];
thetaAlpha = [[];[]];


for index = 1: size(pts_in,2)
    R = norm(pts_in(:,index));
    z = pts_in(3,index);
    
    theta = round(asind(z/R));
    alpha = atan2d(pts_in(2,index),pts_in(1,index));
    thetaAlpha = [thetaAlpha;[theta,alpha]];
end

[M_min,Imin]  = min(thetaAlpha(:,2));
[M_max,Imax]  = max(thetaAlpha(:,2));
if M_max - M_min> 180
    idx = find(thetaAlpha(:,2)<180);
    thetaAlpha(idx,2) = thetaAlpha(idx,2) + 360;
end
for angle = -15:2:15
    thetaidx = find(thetaAlpha(:,1)==angle);
    rings = thetaAlpha(thetaidx,2);
    [M,Imin]  = min(rings);
    [M,Imax]  = max(rings);
    idx_out = [idx_out,thetaidx(Imin),thetaidx(Imax)];
end
end