function Tinit = plane_init(pc_bors_coeff,cam_bors_coeff,pcs_corners,cam_corners)

cam_centers=[];
pc_centers=[];
pc_n=[];
cam_n=[];

for idx=1:size(pcs_corners,2)
    cam_centers =[cam_centers,mean(cam_corners{idx},2)];
    pc_centers =[pc_centers,mean(pcs_corners{idx},2)];
    pc_n =[pc_n,pc_bors_coeff{idx}];
    cam_n=[cam_n,cam_bors_coeff{idx}];
end

Tinit = TransformationSVDSolver(cam_n(1:3,:),pc_n(1:3,:));

t = cam_centers - Tinit(1:3,1:3)*pc_centers;
Tinit(1:3,4) = mean(t,2);
end