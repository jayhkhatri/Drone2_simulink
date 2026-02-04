function [T, X] = ode1_solver(f, tspan, x0, h)
T = tspan(1):h:tspan(2);
N = length(T);
n = length(x0);

X = zeros(n, N);
X(:,1) = x0;

for k = 1:N-1
    dx = f(T(k), X(:,k));
    X(:,k+1) = X(:,k) + h*dx;
end
end
