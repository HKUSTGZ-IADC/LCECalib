function transform = TransformationSVDSolver(pt_ref,pt_mov)
% pt_ref: dim*npts
% pt_mov: dim*npts

q_ref =pt_ref -  sum(pt_ref,2)/size(pt_ref,2);
q_mov =pt_mov -  sum(pt_mov,2)/size(pt_mov,2);

Dim = size(pt_mov,1);
H = zeros(3,3);
for idx = 1:size(q_ref,2)
    H = H + q_mov(:,idx)*q_ref(:,idx)';
end
[U,S,V] = svd(H);

X = V*U';

detX = det(X);
R = [];
if detX-1 > -0.0001
    R = X;
elseif detX<0
    B = eye(Dim);
    B(Dim,Dim) = det(V*U');
    R = V*B*U';
end

T = sum(pt_ref - R*pt_mov,2)/size(pt_ref,2);
transform = [[R,T];[0,0,0,1]];
end

