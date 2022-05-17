% input: cbcorner: 4x3 points. Order:         
%          [[borW/2,borH/2,0]', [borW/2,-borH/2,0]', ...
%          [-borW/2,-borH/2,0]', [-borW/2,borH/2,0]'];
%           X
%      1|---^---|2
%       |   |   |
%     Y <---|   |
%       |       |
%      4|---|---|3
% output: cbedge:     4xN edge points with the same order to cbcorner
%                       [x y z id]: coordinate and edge id
%         cbedge_dir: 6x4 direction vectors
function [cbedge, cbedge_dir] = generateBoardPtsFromCorner(cbcorner)
  line1 = cbcorner(:, 2) - cbcorner(:, 1);
  line2 = cbcorner(:, 3) - cbcorner(:, 2);
  line3 = cbcorner(:, 4) - cbcorner(:, 3);
  line4 = cbcorner(:, 1) - cbcorner(:, 4);
  cbedge = zeros(4, 0);
  for r = 0:0.01:1
    cbedge(:, end + 1) = [cbcorner(:, 1) + r .* line1; 1];
  end
  for r = 0:0.01:1
    cbedge(:, end + 1) = [cbcorner(:, 2) + r .* line2; 2];
  end
  for r = 0:0.01:1
    cbedge(:, end + 1) = [cbcorner(:, 3) + r .* line3; 3];
  end
  for r = 0:0.01:1
    cbedge(:, end + 1) = [cbcorner(:, 4) + r .* line4; 4];
  end  
  
  cbedge_dir = zeros(6, 4);
  cbedge_dir(:, 1) = [line1; cbcorner(:, 1)];
  cbedge_dir(:, 2) = [line2; cbcorner(:, 2)];
  cbedge_dir(:, 3) = [line3; cbcorner(:, 3)];
  cbedge_dir(:, 4) = [line4; cbcorner(:, 4)];
end