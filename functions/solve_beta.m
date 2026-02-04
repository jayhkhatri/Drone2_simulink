function [beta, info] = solve_beta(h_fun, Jh_fun, theta, v, beta0, opts)
%SOLVE_BETA  Solve h(x, beta) = v for beta using Newton + LM + fallback
%
%   Inputs:
%       h_fun  : function handle, h_fun(x, beta) returns 2x1 vector
%       Jh_fun : function handle, Jh_fun(x, beta) returns 2x2 Jacobian wrt beta
%       x      : system state
%       v      : desired virtual input
%       beta0  : warm-start initial guess (2x1)
%       opts   : options struct with fields:
%                   .maxIter     (default 10)
%                   .tol_r       (default 1e-6)
%                   .tol_beta    (default 1e-6)
%                   .lambdaLM0   (default 1e-6)
%                   .regPI       (default 1e-6)
%                   .beta_min    (optional box constraint)
%                   .beta_max
%
%   Outputs:
%       beta   : solved beta
%       info   : struct with fields:
%                   .iterations
%                   .residual
%                   .converged
%                   .usedFallback
%
%   Jay's UAV–AUV Hybrid Control Project (PHD_DRONE)
%   ----------------------------------------------------

%% Default options
if ~isfield(opts,'maxIter');    opts.maxIter  = 10;     end
if ~isfield(opts,'tol_r');      opts.tol_r    = 1e-6;   end
if ~isfield(opts,'tol_beta');   opts.tol_beta = 1e-6;   end
if ~isfield(opts,'lambdaLM0');  opts.lambdaLM0 = 1e-6;  end
if ~isfield(opts,'regPI');      opts.regPI    = 1e-6;   end

beta = beta0;
lambda = opts.lambdaLM0;
usedFallback = false;

%% Iterative solve
for k = 1:opts.maxIter
    
    r = h_fun(theta, beta) - v;           % residual
    if norm(r) < opts.tol_r
        break;
    end
    
    % Jacobian
    J = Jh_fun(theta, beta);
    
    % LM step: (J'J + lambda I) delta = -J' r
    % lambda = max(lambda, 1e-6);   % keeps it from becoming too small

    A = J'*J + lambda * eye(2);
    % A = A+ 1e-8*eye(2); 
    g = J' * r;
    
   if rcond(A) < 1e-12 || any(isnan(A(:))) || any(isinf(A(:)))
       delta = -pinv(A) * g;
   else
       delta = -A \ g;
   end
%% SVD decomposition method
    % [U, S, V] = svd(J, 'econ');
    % S_d = diag(S);
    % S_reg = S_d ./ (S_d.^2 + lambda);   % LM update in SVD domain
    % delta = -V * diag(S_reg) * U' * r;

    
    
    % delta = -A \ g;
    beta_trial = beta + delta;
    
    % Enforce optional bounds
    if isfield(opts,'beta_min')
        beta_trial = max(beta_trial, opts.beta_min);
    end
    if isfield(opts,'beta_max')
        beta_trial = min(beta_trial, opts.beta_max);
    end
    
    % Check residual improvement
    r_trial = h_fun(theta, beta_trial) - v;
    
    if norm(r_trial) < norm(r)       % accept step
        beta = beta_trial;
        lambda = lambda / 10;
        
        if norm(delta) < opts.tol_beta
            break;
        end
        
    else                             % reject step
        lambda = lambda * 10;
        
        % If lambda becomes huge → fallback
        if lambda > 1e6
            usedFallback = true;
            % Regularized pseudo-inverse step
            delta_pi = -(J'*J + opts.regPI*eye(2)) \ (J'*r);
            beta = beta + delta_pi;
            break;
        end
    end
end

%% Prepare info
info.iterations = k;
info.residual = norm(h_fun(theta,beta)-v);
info.converged = info.residual < opts.tol_r;
info.usedFallback = usedFallback;

end
