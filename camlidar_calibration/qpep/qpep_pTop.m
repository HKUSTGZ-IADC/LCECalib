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
% qpep_pTop.m: The globally optimal solution and covariance estimation
%              of point-to-plane registration problem
% input: 
%   r0  (Nx3): target point cloud to be transformed 
%   nvr (Nx3): normal of the target point cloud
%   b0  (Nx3): reference point cloud 
%   nvb (Nx3): normal of the reference point cloud
%   plot_flag: plot figures (true)
% output:
%   Rest     : estimated rotation matrix 
%   test     : estimated translation vector (Rest * r0 + test = b0)
%   covR     : covariance matrix of quaternion
%   covt     : covariance matrix of translation

function [Rest, test, covR, covt, mean_residual] = ...
  qpep_pTop(r0, nvr, b0, nvb, comp_cov, plot_flag, weights)

len = size(r0, 1);

%%
% TODO(jjiao): need to figure out why W0, Q0 are zero
[W0, Q0, D, G, c, coef_f_q_sym, coef_J_pure, coefs_tq, pinvG] = ...
  pTop_WQDGc_new_weight(r0, b0, nvb, weights);
t_funcs = {@t1_pTop_func_new, @t2_pTop_func_new, @t3_pTop_func_new};

W0_ = W0(1 : 3, :);
Q0_ = Q0(1 : 3, :);
W0_(1, :) = W0(1, :) + W0(2, :) + W0(3, :);
W0_(2, :) = W0(2, :) + W0(3, :) + W0(4, :);
W0_(3, :) = W0(3, :) + W0(4, :) + W0(1, :);
Q0_(1, :) = Q0(1, :) + Q0(2, :) + Q0(3, :);
Q0_(2, :) = Q0(2, :) + Q0(3, :) + Q0(4, :);
Q0_(3, :) = Q0(3, :) + Q0(4, :) + Q0(1, :);

% step 1: QPEP_WQ_grobner get initial solution
[~, ~, X] = QPEP_WQ_grobner(W0, Q0, @solver_WQ_approx, ...
  @mon_J_pure_pTop_func_new, t_funcs, coef_J_pure, coefs_tq, pinvG, {[1, 2, 3]});
X_grobner = X;
% disp('X_grobner:')
% disp(num2str(X_grobner, '%.5f '))

% step 2: QPEP_lm_single get refined solution
[~, ~, X_] = QPEP_lm_single(dcm2quat(X(1 : 3, 1 : 3)).', 100, 5e-2, ...
  @eq_pTop_func_new, @Jacob_pTop_func_new, t_funcs, ...
  coef_f_q_sym, coefs_tq, pinvG);
q = dcm2quat(X_(1 : 3, 1 : 3)).';
if(q(1) < 0)
    q = - q;
end

Rest = X_(1:3, 1:3);
test = X_(1:3, 4);

mean_residual = 0;
len = size(r0, 1);
for i = 1 : len
  rr = r0(i, :).';
  bb = b0(i, :).';
  nn = nvb(i, :).';
  mean_residual = mean_residual + abs(weights(i) * nn' * (Rest * rr + test - bb));
end
mean_residual = mean_residual / len;


%% Monte-carlo sampling to compute covariance 
% at each iteration, generate noisy data and compute the transformation
if comp_cov
  num = 1000;
  r_noise = 1e-3;
  b_noise = 1e-3;
  n_noise = 1e-3;
  ys = zeros(9, num);
  vs = zeros(28, num);
  Ds = zeros(3, 28, num);
  ds = zeros(3 * 28, num);
  Gs = zeros(3, 9, num);
  gs = zeros(3 * 9, num);
  cs = zeros(3, num);

  Dvs = zeros(3, num);
  Gys = zeros(3, num);
  DvGys = zeros(3, num);

  qs = zeros(4, num);
  ts = zeros(3, num);
  ps = zeros(3, num);
  Ws = zeros(4, 64, num);
  Qs = zeros(4, 4, num);

  tic;
  for j = 1 : num
      r = r0 + r_noise * randn(len, 3);
      b = b0 + b_noise * randn(len, 3);
      n = nvb + n_noise * randn(len, 3);

      [W, Q, D, G, c, coef_f_q_sym, coef_J_pure, coefs_tq, pinvG] = ...
        pTop_WQDGc_new_weight(r, b, n, weights);
      Ws(:, :, j) = W;
      Qs(:, :, j) = Q;

      R = QPEP_WQ_grobner(W, Q, @solver_WQ_approx, ...
            @mon_J_pure_pTop_func_new, t_funcs, coef_J_pure, coefs_tq, pinvG, {[1, 2, 3]});
      q = dcm2quat(real(R)).';
      if(q(1) < 0)
          q = - q;
      end

      [~, ~, X_] = QPEP_lm_single(q, 100, 5e-2, ...
        @eq_pTop_func_new, @Jacob_pTop_func_new, t_funcs, ...
        coef_f_q_sym, coefs_tq, pinvG);
      q = dcm2quat(X_(1 : 3, 1 : 3)).';
      t = X_(1 : 3, 4);
      if(q(1) < 0)
          q = - q;
      end   

      qs(:, j) = q;
      ts(:, j) = t;
      y = y_func_pTop_new(q);
      v = v_func_pTop_new(q);
      ys(:, j) = y;
      vs(:, j) = v;
      Ds(:, :, j) = D;
      ds(:, j) = vec(D);
      Gs(:, :, j) = G;
      gs(:, j) = vec(G);
      cs(:, j) = c;
      Dvs(:, j) = D * v;
      Gys(:, j) = G * y;
      ps(:, j) = D * v + G * y + c;
      DvGys(:, j) = D * v + G * y;

      if(mod(j, 1000) == 0)
          fprintf('%d\n', j);
      end
  end
  time = toc / num

  Sigma_q_stat = covx(qs.', qs.');
  
  Sigma_t_stat = covx(ts.', ts.');

  Sigma_y_stat = covx(ys.', ys.');
  Sigma_v_stat = covx(vs.', vs.');

  Sigma_d_stat = covx(ds.', ds.');
  Sigma_g_stat = covx(gs.', gs.');
  Sigma_c_stat = covx(cs.', cs.');

  Sigma_d_q_stat = covx(ds.', qs.');
  Sigma_q_d_stat = covx(qs.', ds.');
  Sigma_g_q_stat = covx(gs.', qs.');
  Sigma_q_g_stat = covx(qs.', gs.');
  Sigma_c_q_stat = covx(cs.', qs.');
  Sigma_q_c_stat = covx(qs.', cs.');

  Sigma_d_g_stat = covx(ds.', gs.');
  Sigma_g_d_stat = covx(gs.', ds.');
  Sigma_d_c_stat = covx(ds.', cs.');
  Sigma_c_d_stat = covx(cs.', ds.');
  Sigma_c_g_stat = covx(cs.', gs.');
  Sigma_g_c_stat = covx(gs.', cs.');

  Sigma_p_stat = covx(ps.', ps.');

  Sigma_Dv_stat = covx(Dvs.', Dvs.');
  Sigma_Gy_stat = covx(Gys.', Gys.');
  Sigma_DvGy_stat = covx(DvGys.', DvGys.');

  mean_D = zeros(3, 28);
  mean_d = mean(ds, 2);
  mean_G = zeros(3, 9);
  mean_g = mean(gs, 2);
  mean_Dv = mean(Dvs, 2);
  mean_Gy = mean(Gys, 2);
  mean_W = zeros(4, 64);
  mean_Q = zeros(4, 4);

  for i = 1 : num
      mean_D = mean_D + 1 / num * Ds(:, :, i);
      mean_G = mean_G + 1 / num * Gs(:, :, i);
      mean_W = mean_W + 1 / num * Ws(:, :, i);
      mean_Q = mean_Q + 1 / num * Qs(:, :, i);
  end

  len_ = len;

  r = r0 + r_noise * randn(len, 3);
  b = b0 + b_noise * randn(len, 3);
  n = nvb + n_noise * randn(len, 3);

  %% compute covariance based on the Monte-Carlo sampling
  y = y_func_pTop_new(q);
  v = v_func_pTop_new(q);
  V_cal = kron(v.', eye(3));
  Y_cal = kron(y.', eye(3));

  syms q0_ q1_ q2_ q3_
  q_ = [q0_; q1_; q2_; q3_];
  y_ = y_func_pTop_new(q_);
  v_ = v_func_pTop_new(q_);

  partial_v_q_sym = jacobian(v_, q_);
  partial_v_q_sym_func = matlabFunction(partial_v_q_sym, 'Vars', {q_});
  partial_v_q_sym_val = partial_v_q_sym_func(q);

  partial_y_q_sym = jacobian(y_, q_);
  partial_y_q_sym_func = matlabFunction(partial_y_q_sym, 'Vars', {q_});
  partial_y_q_sym_val = partial_y_q_sym_func(q);

  F = D * partial_v_q_sym_val + G * partial_y_q_sym_val;

  cov_left = V_cal * Sigma_d_stat * V_cal.' + Y_cal * Sigma_g_stat * Y_cal.' + Sigma_c_stat + ...
             V_cal * Sigma_d_g_stat * Y_cal.' + V_cal * Sigma_d_c_stat + Y_cal * Sigma_g_c_stat + ...
             Sigma_c_d_stat * V_cal.' + Sigma_c_g_stat * Y_cal.' + Y_cal * Sigma_g_d_stat * V_cal.';


  scalings = 1e3;
  solver = 'sedumi';
  verbose = true;
  for i = 1 : length(scalings)
      scaling = scalings(i);

      epsX = scaling * 1e-23;
      epsQuat = scaling * 1e-15;


      [XX, ff] = optimize_quat_cov2(q, F, scaling * cov_left, solver, verbose);
      ff = ff / scaling^2
      scale = abs(norm(cov_left, 'fro') / norm(F * XX * F.', 'fro'));
      XX * scale;
      Sigma_q_stat;

      if plot_flag
        figure(i);
        plot_q_cov(Sigma_q_stat, XX * scale, qs);  % plot stat_cov, est_cov, est_q at multiple iterations
        if(~ispc())
            set(gcf, 'Unit', 'Centimeters', 'Position', 6 * [5.5 5 5.5 5])
        end
      end
  end

  covR = Sigma_q_stat;
  covt = Sigma_t_stat;
else
  covR = zeros(4, 4);
  covt = zeros(3, 3);
end
end






