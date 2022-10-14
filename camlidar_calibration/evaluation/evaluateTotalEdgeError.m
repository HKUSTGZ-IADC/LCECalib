% input: 
% output: err: the total error by summing all absolute planar error terms
%         cnt: number of planar error terms
function [err, cnt] = evaluateTotalEdgeError(T_est, cbcorner, lepts)
  err = 0;
  cnt = 0;
  %%%%% add edge residuals
  [cbedge, cbedge_dir] = generateBoardPtsFromCorner(cbcorner);
  lepts_cam = T_est(1:3, 1:3) * lepts + T_est(1:3, 4);  % 3xN
  corre_idx = knnsearch(cbedge(1:3, :)', lepts_cam', 'K', 1);
  corre_cbedge = cbedge(:, corre_idx); % 4xN
  for j = 1:size(lepts_cam, 2)
    p = lepts_cam(:, j);
    p1 = cbedge_dir(4:6, corre_cbedge(4, j));
    p2 = p1 + cbedge_dir(1:3, corre_cbedge(4, j));
    p1p2 = p1 - p2;
    p1p = p1 - p;
    p2p = p2 - p;
    n1 = cross(p1p, p2p);
    n1 = n1 / norm(n1);
    n2 = cross(n1, p1p2);
    n2 = n2 / norm(n2);
    err = err + ...
      abs(n1' * (p - corre_cbedge(1:3, j))) + ...
      abs(n2' * (p - corre_cbedge(1:3, j)));
    cnt = cnt + 2;
  end
end