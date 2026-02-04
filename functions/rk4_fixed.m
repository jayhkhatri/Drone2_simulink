function [T, X] = rk4_fixed(f, tspan, x0, h)
% RK4_FIXED  Classic 4th-order Runge–Kutta solver (fixed step)
%
% Inputs:
%   f      - function handle f(t,x)
%   tspan  - [t0 tf]
%   x0     - initial state (nx1)
%   h      - step size
%
% Outputs:
%   T      - time vector
%   X      - state trajectory (n x length(T))

t0 = tspan(1);
tf = tspan(2);

T = t0:h:tf;
N = length(T);
n = length(x0);

X = zeros(n, N);
X(:,1) = x0;

for k = 1:N-1
    t = T(k);
    x = X(:,k);

    k1 = f(t,        x);
    k2 = f(t + h/2,  x + h*k1/2);
    k3 = f(t + h/2,  x + h*k2/2);
    k4 = f(t + h,    x + h*k3);

    X(:,k+1) = x + h*(k1 + 2*k2 + 2*k3 + k4)/6;
end
end
