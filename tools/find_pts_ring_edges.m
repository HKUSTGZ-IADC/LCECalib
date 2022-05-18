% Output: 
function idx_out = find_pts_ring_edges(pts_in)
% using theta information  
if (size(pts_in, 1) == 3)
  % pts_in dim*npts
  % find edge points for 16 lidar
  idx_out =[];
  minRange = -15;
  maxRange = 15;
  intRange = 2;  
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
  for angle = minRange:intRange:maxRange
    thetaidx = find(thetaAlpha(:,1) == angle);
    rings = thetaAlpha(thetaidx,2);
    [M,Imin]  = min(rings);
    Imin = rings<=(M+0.1);
    [M,Imax]  = max(rings);
    Imax = rings>=(M-0.1);
    idx_out = [idx_out,thetaidx(Imin),thetaidx(Imax)];
  end  
% using ring information  
else 
  idx_out =[];
  minRange = min(pts_in(4, :));
  maxRange = max(pts_in(4, :));
  intRange = 1;
  Alpha = [];
  for index = 1:size(pts_in, 2)
      alpha = atan2d(pts_in(2, index), pts_in(1, index));
      Alpha = [Alpha, alpha];
  end
  [M_min,Imin]  = min(Alpha);
  [M_max,Imax]  = max(Alpha);
  if M_max - M_min > 180
      idx = find(Alpha < 180);
      Alpha(idx) = Alpha(idx) + 360;
  end
  for ring = minRange:intRange:maxRange
    ringidx = find(pts_in(4, :) == ring);
    ringAngle = Alpha(ringidx);
    [M, Imin]  = min(ringAngle);  % the idx of the minimun angle
    Imin = ringAngle<=(M+0.1);
    [M, Imax]  = max(ringAngle);
    Imax = ringAngle>=(M-0.1);
    idx_out = [idx_out, ringidx(Imin), ringidx(Imax)];
  end
end
end