% 
% LibQPEP: A Library for Globally Optimal Solving Quadratic Pose Estimation Problems (QPEPs),
%          It also gives highly accurate uncertainty description of the solutions.
%
%
% Article: 
%      Wu, J., Zheng, Y., Gao, Z., Jiang, Y., Hu, X., Zhu, Y., Jiao, J., Liu, M. (2020)
%           Quadratic Pose Estimation Problems: Globally Optimal Solutions, 
%           Solvability/Observability Analysis and Uncertainty Description.
%           IEEE Transactions on Robotics.
%           https://doi.org/10.1109/TRO.2022.3155880
%
%
% Authors:      Jin Wu and Ming Liu
% Affiliation:  Hong Kong University of Science and Technology (HKUST)
% Emails:       jin_wu_uestc@hotmail.com; eelium@ust.hk
% Websites:     https://zarathustr.github.io
%               https://ram-lab.com
%
%
% qpep_pnp.m: The globally optimal solution and covariance estimation
%             of the PnP problem
% input: 
%   image_pt0  (Nx2): 2D points in an image to be transformed
%   world_pt0  (Nx3): 3D points in the world
%   K          (3x3): intrinsic camera matrix
%   plot_flag: plot figures (true)
% output:
%   Rest     : estimated rotation matrix 
%   test     : estimated translation vector 
%                   (K * (Rest * image_pt0 + test) = world_pt0)
%   covR     : covariance matrix of quaternion
%   covt     : covariance matrix of translation

function [Rest, test, covR, covt] = qpep_pnp(image_pt0, world_pt0, K, plot_flag)

use_p3p = false;
len = size(world_pt0, 1);

%% implement the QPEP-PNP
problem_scale = 1e-5;
[W0, Q0, ~, coef_f_q_sym, coef_J_pure, coefs_tq, pinvG] = ...
  pnp_WQD_new(image_pt0, world_pt0, K, problem_scale);
t_funcs = {@t1_pnp_func_new, @t2_pnp_func_new, @t3_pnp_func_new};

W0_ = W0(1 : 3, :);
Q0_ = Q0(1 : 3, :);
W0_(1, :) = W0(1, :) + W0(2, :) + W0(3, :);
W0_(2, :) = W0(2, :) + W0(3, :) + W0(4, :);
W0_(3, :) = W0(3, :) + W0(4, :) + W0(1, :);
Q0_(1, :) = Q0(1, :) + Q0(2, :) + Q0(3, :);
Q0_(2, :) = Q0(2, :) + Q0(3, :) + Q0(4, :);
Q0_(3, :) = Q0(3, :) + Q0(4, :) + Q0(1, :);

[~, ~, X] = QPEP_WQ_grobner(W0_, Q0_, @solver_WQ_1_2_3_4_5_9_13_17_33_49_approx, ...
                            @mon_J_pure_pnp_func_new, t_funcs, coef_J_pure, coefs_tq, pinvG, {[1, 2, 3]});
X_grobner = X;
disp('X_grobner:')
disp(num2str(X_grobner, '%.5f '))

[~, ~, X_] = QPEP_lm_single(dcm2quat(X(1 : 3, 1 : 3)).', 1000, 5e-2, ...
                          @eq_pnp_func_new, @Jacob_pnp_func_new, t_funcs, ...
                          coef_f_q_sym, coefs_tq, pinvG);

J_pnp_loss(image_pt0, world_pt0, K, X_(1 : 3, 1 : 3), X_(1 : 3, 4))
% J_pnp_loss(image_pt0, world_pt0, K, R0, t0)

% disp('X_est:')
% disp(num2str(X_, '%.5f '))

Rest = X_(1:3, 1:3);
test = X_(1:3, 4);

%% compute covariance
num = 100;
world_noise = 1e-2;
image_noise = 1e-0;

v1s = zeros(37, num);
v2s = zeros(37, num);
v3s = zeros(37, num);
Ds = zeros(3, 37, num);
ds = zeros(3 * 37, num);
cs = zeros(3, num);

Dvs = zeros(3, num);

qs = zeros(4, num);
ts = zeros(3, num);
ps = zeros(3, num);
Ws = zeros(4, 64, num);
Qs = zeros(4, 4, num);
for j = 1 : num
    world_pt = world_pt0;
    image_pt = image_pt0;
    for i = 1 : len
        world_pt(i, :) = world_pt(i, :) + world_noise * randn(1, 3);
        image_pt(i, :) = image_pt(i, :) + image_noise * randn(1, 2);
    end
    
    [W, Q, D, coef_f_q_sym, coef_J_pure, coefs_tq, pinvG] = pnp_WQD_new(image_pt, world_pt, K, problem_scale);
    
    if(use_p3p)
        R = estimateWorldCameraPoseNew(image_pt, world_pt, camParams);
        R = R.';
    else
        perms = [
                    1, 2, 3;
                    1, 3, 4;
                    1, 2, 4;
                    2, 3, 4;
                    ];
                
        X_QPEPs = zeros(4, 4, 4);
        fvals = zeros(4, 1);
        for i = 1 : 4
            [~, ~, X_QPEP, fval] = QPEP_WQ_grobner(W0, Q0, @solver_WQ_1_2_3_4_5_9_13_17_33_49_approx, ...
                                                   @mon_J_pure_pnp_func_new, t_funcs, coef_J_pure, ...
                                                   coefs_tq, pinvG, {perms(i, :)});
            X_QPEPs(:, :, i) = X_QPEP;
            fvals(i) = fval(1);
        end
        [~, idx] = sort(fvals);
        X_QPEP = X_QPEPs(:, :, idx(1));
        R = X_QPEP(1 : 3, 1 : 3);
    end
    q = dcm2quat(real(R)).';
    if(q(1) < 0)
        q = - q;
    end
    
    Ws(:, :, j) = W;
    Qs(:, :, j) = Q;
    [~, ~, X_] = QPEP_lm_single(q, 1000, 5e-2, ...
                          @eq_pnp_func_new, @Jacob_pnp_func_new, t_funcs, ...
                          coef_f_q_sym, coefs_tq, pinvG);
    q = dcm2quat(X_(1 : 3, 1 : 3)).';
    if(q(1) < 0)
        q = - q;
    end
    t = X_(1 : 3, 4);
    
    qs(:, j) = q;
    ts(:, j) = t;
    v1 = v1_func_pnp_new(q);
    v1s(:, j) = v1;
    v2 = v2_func_pnp_new(q);
    v2s(:, j) = v2;
    v3 = v3_func_pnp_new(q);
    v3s(:, j) = v3;
    Ds(:, :, j) = D;
    d1 = D(1, :).';
    d2 = D(2, :).';
    d3 = D(3, :).';
    ds(:, j) = vec(D);
    Dvs(:, j) = [
        d1.' * v1;
        d2.' * v2;
        d3.' * v3;
        ];
    ps(:, j) = [
        d1.' * v1;
        d2.' * v2;
        d3.' * v3;
        ];
    
    if(mod(j, 1000) == 0)
        fprintf('%d\n', j);
    end
end

Sigma_q_stat = covx(qs.', qs.');
Sigma_t_stat = covx(ts.', ts.');  % TODO:

Sigma_v1_stat = covx(v1s.', v1s.');
Sigma_v2_stat = covx(v2s.', v2s.');
Sigma_v3_stat = covx(v3s.', v3s.');

Sigma_d_stat = covx(ds.', ds.');

Sigma_d_q_stat = covx(ds.', qs.');
Sigma_q_d_stat = covx(qs.', ds.');

Sigma_p_stat = covx(ps.', ps.');
Sigma_Dv_stat = covx(Dvs.', Dvs.');

mean_D = zeros(3, 37);
mean_d = mean(ds, 2);
mean_Dv = mean(Dvs, 2);
mean_W = zeros(4, 64);
mean_Q = zeros(4, 4);

for i = 1 : num
    mean_D = mean_D + 1 / num * Ds(:, :, i);
    mean_W = mean_W + 1 / num * Ws(:, :, i);
    mean_Q = mean_Q + 1 / num * Qs(:, :, i);
end

world_pt = world_pt0;
image_pt = image_pt0;
for i = 1 : len
    world_pt(i, :) = world_pt(i, :) + world_noise * randn(1, 3);
    image_pt(i, :) = image_pt(i, :) + image_noise * randn(1, 2);
end
    
[W, Q, D, coef_f_q_sym, coef_J_pure, coefs_tq, pinvG] = pnp_WQD_new(image_pt, world_pt, K, problem_scale);
if(use_p3p)
    R = estimateWorldCameraPoseNew(image_pt, world_pt, camParams);
    R = R.';
else
    R = QPEP_WQ_grobner(W, Q, @solver_WQ_1_2_3_4_5_9_13_17_33_49_approx, ...
                        @mon_J_pure_pnp_func_new, t_funcs, coef_J_pure, ...
                        coefs_tq, pinvG, {[1, 2, 3]});
end

q = dcm2quat(real(R)).';
if(q(1) < 0)
    q = - q;
end
    
[~, ~, X_] = QPEP_lm_single(q, 1000, 5e-2, ...
                          @eq_pnp_func_new, @Jacob_pnp_func_new, t_funcs, ...
                          coef_f_q_sym, coefs_tq, pinvG);
q = dcm2quat(X_(1 : 3, 1 : 3)).';
if(q(1) < 0)
    q = - q;
end


syms q0_ q1_ q2_ q3_
DD = sym('DD', [3 37]);
q_ = [q0_; q1_; q2_; q3_];
v1_ = v1_func_pnp_new(q_);
v2_ = v2_func_pnp_new(q_);
v3_ = v3_func_pnp_new(q_);
p_ = [
    DD(1, :) * v1_;
    DD(2, :) * v2_;
    DD(3, :) * v3_;
];

partial_p_q_sym = jacobian(p_, q_);
partial_p_q_sym_func = matlabFunction(partial_p_q_sym, 'Vars', {q_, DD});
partial_p_q_sym_val = partial_p_q_sym_func(q, D);

partial_p_d_sym = jacobian(p_, vec(DD));
partial_p_d_sym_func = matlabFunction(partial_p_d_sym, 'Vars', {q_});
partial_p_d_sym_val = partial_p_d_sym_func(q);


partial_v1_q_sym = jacobian(v1_, q_);
partial_v1_q_sym_func = matlabFunction(partial_v1_q_sym, 'Vars', {q_});
partial_v1_q_sym_val = partial_v1_q_sym_func(q);
partial_v2_q_sym = jacobian(v2_, q_);
partial_v2_q_sym_func = matlabFunction(partial_v2_q_sym, 'Vars', {q_});
partial_v2_q_sym_val = partial_v2_q_sym_func(q);
partial_v3_q_sym = jacobian(v3_, q_);
partial_v3_q_sym_func = matlabFunction(partial_v3_q_sym, 'Vars', {q_});
partial_v3_q_sym_val = partial_v3_q_sym_func(q);

F = [
    D(1, :) * partial_v1_q_sym_val; 
    D(2, :) * partial_v2_q_sym_val; 
    D(3, :) * partial_v3_q_sym_val;
    ];
cov_left = partial_p_d_sym_val * Sigma_d_stat * partial_p_d_sym_val.';


scalings = [1e3, 3e1, 1, 1e-1];
solver = 'sedumi';
verbose = false;
for i = 1 : length(scalings)
    num_ = len;
    scaling = scalings(i);
    AA = zeros(num_ * 9, 16);
    bb = zeros(num_ * 9, 1);
    problem_scale = 1e-12;
    for j = 1 : num_
        D_ = pnp_D(image_pt, world_pt, K, problem_scale);
        F_ = partial_p_q_sym_func(q, D_);
        right2 = partial_p_d_sym_val * Sigma_d_stat * partial_p_d_sym_val.';
        right2 = scaling * right2;
        bb(9 * (j - 1) + 1 : 9 * j) = vec(right2);
        AA(9 * (j - 1) + 1 : 9 * j, :) = kron(F_, F_);
    end

    epsX = scaling * 1e-15;
    epsQuat = scaling * 1e-15;
    
    [XX, ff] = optimize_quat_cov(AA, bb, q, F, scaling * cov_left, epsX, epsQuat, solver, verbose);
    ff = ff / scaling^2
    scale = abs(norm(cov_left, 'fro') / norm(F * XX * F.', 'fro'));
    XX * scale;
    Sigma_q_stat;

    if plot_flag
      figure(i);
      str = sprintf('Scaling = %e', scaling);
      plot_q_cov(Sigma_q_stat, XX * scale, qs);
      sgtitle(str);
    end
end

if plot_flag
  figure(length(scalings) + 1);
  [XX, ff] = optimize_quat_cov2(q, F, 1e3 * cov_left, solver, verbose);
  scale = abs(norm(cov_left, 'fro') / norm(F * XX * F.', 'fro'));
  plot_q_cov(Sigma_q_stat, XX * scale, qs);
  sgtitle('Simplified Optimization');
end

covR = Sigma_q_stat;
covt = Sigma_t_stat;

end


