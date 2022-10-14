function Tf_out = plane_error_minimize(plane_coeffs,pc_boards,tf0)
% plane_coeffs: camera plane equation： [a;b;c;d],ax+by+cz+d = 0
% pc_boards: points from lidar. It's a cell data, 

q = rotm2quat(tf0(1:3,1:3));
theta = 2*acos(q(1));
if theta==0
    lie_vec =[0,0,0];
else
    lie_vec = theta*q(2:4)./sin(theta/2);    
end


x0 = [double(rotm2eul(tf0(1:3,1:3))),double(tf0(1:3,4)')]; % lie_x,lie_y,lie_z,tx,ty,tz
F = @(x) obj_func(x(1:3),x(4:6),plane_coeffs,pc_boards);
opts = optimoptions('fminunc','Algorithm','quasi-newton','Display','iter','MaxFunctionEvaluations', 1000);
[x_optm,fval] = fminunc(F,x0,opts);

R = eul2rotm(x_optm(1:3));
Tf_out = [[R,x_optm(4:6)'];[0,0,0,1]];
end


function error = obj_func(lievec,tvec,plane_coeffs,pc_boards)
% R = SO3.exp(lievec);
% R = R.R;
R = eul2rotm(lievec);
error = 0;
for idx = 1: size(plane_coeffs,2)
    
    plane_coeff = plane_coeffs{idx};
    plane_pts = pc_boards{idx};
    plane_pts_aft = R * plane_pts + tvec';
    pts_dis = plane_coeff(1:3)*plane_pts_aft+plane_coeff(4);
    error = error + norm(pts_dis)*norm(pts_dis)/size(pts_dis,2);
end
error = double(error)/size(plane_coeffs,2);
end