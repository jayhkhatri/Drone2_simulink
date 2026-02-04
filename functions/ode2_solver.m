function [T, X] = ode2_solver(f, tspan, x0, h)
%% Heun / modified Euler


T = tspan(1):h:tspan(2);
N = length(T);
n = length(x0);

X = zeros(n,N);
X(:,1) = x0;

for k = 1:N-1
    k1 = f(T(k), X(:,k));
    x_pred = X(:,k) + h*k1;
    k2 = f(T(k+1), x_pred);
    X(:,k+1) = X(:,k) + 0.5*h*(k1 + k2);
end
end
