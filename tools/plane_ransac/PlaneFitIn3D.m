function coeff = PlaneFitIn3D(pts)

% abc = -pinv(pts)*ones(size(pts,1),1);
abc = pts\ (-ones(size(pts,1),1));
coeff = [abc;1];
norm_abc = norm(abc);
coeff = coeff/norm_abc;
end

function error = Plane3dVal(model_coeff,points)
% points: dim*npts
points = [points';ones(1,size(points,1))];
norm_abc = norm(model_coeff(1:3));
model_coeff = model_coeff/norm_abc;

diss = abs(model_coeff'*points);
error = diss';
end
