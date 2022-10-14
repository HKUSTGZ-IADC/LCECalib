function pts_out = f_add_noise_dis(pts_in,sigma)
pts_out = pts_in;
mu = [1 2];
z = randn(size(pts_in,2),1)*sigma;
for idx=1:size(pts_in,2)
    dir = pts_in(:,idx)./norm(pts_in(:,idx));
    pts_out(:,idx) = pts_in(:,idx)+z(idx)*dir;
end
end