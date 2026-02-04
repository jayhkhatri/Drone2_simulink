%% LM_vs_PseudoInv_demo.m
% Side-by-side comparison of Levenberg-Marquardt (adaptive lambda)
% vs fixed-regularization pseudo-inverse (Tikhonov gamma).
% Run in MATLAB.

clear; close all; clc;

% --- Problem definition (nonlinear 2->2 mapping) ---
% We'll use a toy h(beta) that is nonlinear and can produce
% ill-conditioning in Jacobian near certain betas.
h_fun = @(beta) [ beta(1) + 0.1*beta(2).^2;
                  2*beta(1).^2 + beta(2) ];

% Jacobian wrt beta (2x2)
Jh_fun = @(beta) [ 1,        0.2*beta(2);
                    4*beta(1), 1 ];

% Choose a target v such that solution exists (or slightly inconsistent)
beta_true = [0.8; -0.5];                  % ground-truth beta (for testing)
v_target = h_fun(beta_true);              % exact target (so exact solution exists)

% Now perturb v to create a slightly inconsistent problem (uncomment to test)
% v_target = v_target + [0.0; 0.05];

% --- LM parameters ---
maxIter = 20;
tol_r = 1e-9;
lambda = 1e-3;      % initial damping
nu = 10;            % lambda up/down factor

% --- fixed-regularization (pseudo-inverse) parameters ---
gamma = 1e-4;       % fixed Tikhonov parameter for fallback solver

% --- warm starts ---
beta_LM = [0; 0];   % initial guess
beta_PI = beta_LM;  % warm start same for fairness

% Storage
res_LM = zeros(maxIter,1);
res_PI = zeros(maxIter,1);
beta_hist_LM = zeros(2,maxIter);
beta_hist_PI = zeros(2,maxIter);
lambda_hist = zeros(maxIter,1);

%% LM iterative loop (adaptive lambda)
for k = 1:maxIter
    r = h_fun(beta_LM) - v_target;          % residual (2x1)
    res_LM(k) = norm(r);
    beta_hist_LM(:,k) = beta_LM;
    lambda_hist(k) = lambda;
    if res_LM(k) < tol_r
        res_LM = res_LM(1:k);
        beta_hist_LM = beta_hist_LM(:,1:k);
        lambda_hist = lambda_hist(1:k);
        break;
    end

    J = Jh_fun(beta_LM);
    A = J.'*J + lambda * eye(2);
    g = J.' * r;
    delta = - A \ g;                        % LM step (Gauss-Newton <-> gradient)
    beta_trial = beta_LM + delta;
    r_trial = h_fun(beta_trial) - v_target;

    if norm(r_trial) < norm(r)              % accept step
        beta_LM = beta_trial;
        lambda = lambda / nu;               % decrease damping (trust GN)
    else                                   % reject step
        lambda = lambda * nu;               % increase damping (be conservative)
        % do not update beta_LM (try again next iter with larger lambda)
    end
end

%% Fixed-regularization pseudo-inverse iterations (single-step style repeated)
% We simulate doing a regularized Gauss-Newton update repeatedly with fixed gamma.
for k = 1:maxIter
    r_pi = h_fun(beta_PI) - v_target;
    res_PI(k) = norm(r_pi);
    beta_hist_PI(:,k) = beta_PI;
    if res_PI(k) < tol_r
        res_PI = res_PI(1:k);
        beta_hist_PI = beta_hist_PI(:,1:k);
        break;
    end

    J = Jh_fun(beta_PI);
    % Regularized pseudo-inverse (Tikhonov)
    delta_pi = - (J.'*J + gamma*eye(2)) \ (J.' * r_pi);
    beta_PI = beta_PI + delta_pi;
end

%% Plot residuals
figure('Name','Residuals per iteration');
it_LM = 1:size(res_LM,1);
it_PI = 1:size(res_PI,1);
semilogy(it_LM, res_LM, '-o','LineWidth',1.6); hold on;
semilogy(it_PI, res_PI, '-s','LineWidth',1.6);
grid on;
xlabel('Iteration'); ylabel('Residual norm ||h(beta)-v||');
legend('LM (adaptive \lambda)','Fixed \gamma (Tikhonov)','Location','best');
title('LM vs Fixed-regularization pseudo-inverse');

% Plot beta trajectories
figure('Name','Beta trajectories');
plot(beta_hist_LM(1,:),'-o','LineWidth',1.4); hold on;
plot(beta_hist_LM(2,:),'-o','LineWidth',1.4);
plot(beta_hist_PI(1,:),'--s','LineWidth',1.4);
plot(beta_hist_PI(2,:),'--s','LineWidth',1.4);
yline(beta_true(1),'k:','beta_{true,1}'); yline(beta_true(2),'k:','beta_{true,2}');
legend('LM beta1','LM beta2','PI beta1','PI beta2','Location','best');
xlabel('Iteration'); ylabel('beta values'); grid on;
title('Trajectories of beta (LM vs fixed-\gamma PI)');

% Print final results
fprintf('LM final beta = [%.6f, %.6f], residual = %.3e, final lambda = %.3e\n', ...
    beta_LM(1), beta_LM(2), res_LM(end), lambda_hist(end));
fprintf('PI final beta = [%.6f, %.6f], residual = %.3e, gamma = %.3e\n', ...
    beta_PI(1), beta_PI(2), res_PI(end), gamma);

%% Demonstrate effect of gamma on conditioning & solution (linearized example)
% Create a Jacobian at a near-singular beta to show effect of gamma
beta_test = [0.01; 5];   % pick point where J may be ill-conditioned
J_test = Jh_fun(beta_test);
svals = svd(J_test);
cond_J = svals(1)/svals(end);
fprintf('At beta_test, singular values = [%.3e, %.3e], cond(J) = %.3e\n', svals(1), svals(2), cond_J);

% Solve a linear least-squares step r = h - v (assume r given)
r_lin = h_fun(beta_test) - v_target;
% Unregularized delta (danger if J'J nearly singular)
delta_unreg = - (J_test.'*J_test) \ (J_test.'*r_lin);   % may blow up or be unstable
delta_reg_small = - (J_test.'*J_test + 1e-6*eye(2)) \ (J_test.'*r_lin);
delta_reg_large = - (J_test.'*J_test + 1e-2*eye(2)) \ (J_test.'*r_lin);
fprintf('delta unreg = [%.3e, %.3e]\n', delta_unreg(1), delta_unreg(2));
fprintf('delta reg small gamma=1e-6 = [%.3e, %.3e]\n', delta_reg_small(1), delta_reg_small(2));
fprintf('delta reg large gamma=1e-2 = [%.3e, %.3e]\n', delta_reg_large(1), delta_reg_large(2));
