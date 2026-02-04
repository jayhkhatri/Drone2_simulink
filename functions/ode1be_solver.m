function [T, X] = ode1be_solver(f, tspan, x0, h)
%% ODE1BE (Backward Euler – Newton solver)


T = tspan(1):h:tspan(2);
N = length(T);
n = length(x0);

X = zeros(n,N);
X(:,1) = x0;

for k = 1:N-1
    tk1 = T(k+1);
    xk = X(:,k);

    % initial guess = forward Euler
    xg = xk + h*f(tk1, xk);

    for iter = 1:20
        g = xg - xk - h*f(tk1, xg);

        % numerical Jacobian
        J = zeros(n);
        eps = 1e-6;
        for i = 1:n
            e = zeros(n,1); e(i)=eps;

            g_pe = (xg+e) - xk - h*f(tk1, xg+e);  % g(x + eps*ei)
            g_me = (xg-e) - xk - h*f(tk1, xg-e);  % g(x - eps*ei)
            J(:,i) = (g_pe - g_me)/(2*eps);       % central diff
            % J(:,i) = ( (xg+e) - xk - h*f(tk1, xg+e) - g )/eps;
        end

        % if rcond(J) < 1e-12
        %    dx = -pinv(J)*g;
        % else
        %    dx = -J\g;
        % end

        % if rcond(J) <1e-3
             lambda = 1e-3;
             dx = -pinv(J + lambda*eye(n))* g;
        % else 
             % dx = -pinv(J)\g;
        % end

        xg = xg + dx;

        if norm(dx) < 1e-7
            break;
        end
    end

    X(:,k+1) = xg;
end
end
