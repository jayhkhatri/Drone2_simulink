% adaptive_dob_example.m
% Minimal working example: Adaptive backstepping + ESO(DOB) for scalar plant.
% Save and run in MATLAB: >> adaptive_dob_example

function adaptive_dob_example
    close all; clear; clc;

    % Simulation time
    Tfinal = 30;
    tspan = [0 Tfinal];

    % ----- Design parameters (tune these) -----
    k1 = 0.8;          % virtual control gain (> 1/sqrt(3) recommended)
    k2 = 1.5;          % damping gain
    gamma = 5;         % adaptation gain

    % ESO (DOB) gains (choose sufficiently large for bandwidth but watch noise)
    ell1 = 30;
    ell2 = 200;
    ell3 = 300;

    % Robustifier (saturation) parameters (optional)
    rho = 0.02;
    eps_sat = 0.05;

    % Projection bounds for theta estimate
    theta_min = -10;
    theta_max = 10;

    % ----- True plant and regressor -----
    theta_true = 1.2;                                 % unknown constant
    f_func = @(x1,x2) (1 + 0.5*tanh(0.5*x1));         % example bounded regressor

    % ----- Disturbance (bounded, smooth) -----
    d_func = @(t) 0.25*sin(0.6*t);                    % amplitude 0.25

    % ----- Initial conditions: [x1;x2; hatx1;hatx2; hatd; hat_theta]
    x1_0 = 0.5;
    x2_0 = 0.0;
    hatx1_0 = 0.0;
    hatx2_0 = 0.0;
    hatd_0  = 0.0;
    hattheta_0 = 0.5;
    x0 = [x1_0; x2_0; hatx1_0; hatx2_0; hatd_0; hattheta_0];

    % Pack params for ODE
    params.k1 = k1; params.k2 = k2; params.gamma = gamma;
    params.ell1 = ell1; params.ell2 = ell2; params.ell3 = ell3;
    params.rho = rho; params.eps_sat = eps_sat;
    params.theta_true = theta_true;
    params.f_func = f_func;
    params.d_func = d_func;
    params.theta_min = theta_min; params.theta_max = theta_max;

    % ODE solver
    opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
    [T,X] = ode45(@(t,x) closed_loop_ode(t,x,params), tspan, x0, opts);

    % Extract signals
    x1 = X(:,1); x2 = X(:,2);
    hatx1 = X(:,3); hatx2 = X(:,4);
    hatd = X(:,5); hattheta = X(:,6);

    % For plotting compute control, z2, f, disturbance, residual
    N = length(T);
    u = zeros(N,1); e_d = zeros(N,1); z2vec = zeros(N,1); fvals = zeros(N,1); dvec = zeros(N,1);
    for i=1:N
        t = T(i);
        xi = X(i,1:2)';
        ht = X(i,6);
        fval = f_func(xi(1),xi(2));
        z2 = xi(2) + k1*xi(1);
        u(i) = -ht * fval - k2*z2 - hatd(i) - rho*saturate(z2/eps_sat);
        e_d(i) = d_func(t) - hatd(i);
        z2vec(i) = z2;
        fvals(i) = fval;
        dvec(i) = d_func(t);
    end

    % ----- Plots -----
    figure('Name','Adaptive backstepping + DOB','Position',[200 200 900 700]);
    subplot(3,2,1);
    plot(T,x1,'LineWidth',1.3); hold on; yline(0,'--k'); grid on;
    title('x_1 (position)'); xlabel('t (s)'); ylabel('x_1');

    subplot(3,2,2);
    plot(T,x2,'LineWidth',1.3); grid on;
    title('x_2 (velocity)'); xlabel('t (s)'); ylabel('x_2');

    subplot(3,2,3);
    plot(T,hattheta,'LineWidth',1.3); hold on; yline(theta_true,'--r'); grid on;
    title('\hat\theta (estimate)'); xlabel('t (s)'); ylabel('\hat\theta'); legend('\hat\theta','\theta_{true}');

    subplot(3,2,4);
    plot(T,hatd,'LineWidth',1.3); hold on; plot(T,dvec,'--','LineWidth',1); grid on;
    title('\hat d and true d'); xlabel('t (s)'); ylabel('d'); legend('\hat d','d');

    subplot(3,2,5);
    plot(T,u,'LineWidth',1.3); grid on;
    title('Control input u'); xlabel('t (s)'); ylabel('u');

    subplot(3,2,6);
    plot(T,e_d,'LineWidth',1.3); grid on;
    title('DOB residual e_d = d - \hat d'); xlabel('t (s)'); ylabel('e_d');

    figure('Name','z2 and robustifier','Position',[250 250 600 300]);
    plot(T,z2vec,'LineWidth',1.3); hold on;
    plot(T, rho*sign(z2vec).*(abs(z2vec)>eps_sat),'--','LineWidth',1);
    title('z_2 and robustifier action'); xlabel('t (s)'); grid on; legend('z_2','robustifier approx');

    % -------------------------
    % ODE function describing closed-loop dynamics
    function xdot = closed_loop_ode(t,x,par)
        % unpack
        x1 = x(1); x2 = x(2);
        hatx1 = x(3); hatx2 = x(4);
        hatd  = x(5); hattheta = x(6);

        k1 = par.k1; k2 = par.k2; gamma = par.gamma;
        ell1 = par.ell1; ell2 = par.ell2; ell3 = par.ell3;
        rho = par.rho; eps_sat = par.eps_sat;
        f = par.f_func(x1,x2);
        d = par.d_func(t);

        % backstepping coords
        z1 = x1;
        z2 = x2 + k1*x1;

        % control law
        u = -hattheta * f - k2 * z2 - hatd - rho * saturate(z2/eps_sat);

        % plant dynamics
        x1dot = x2;
        x2dot = par.theta_true * f + u + d;

        % ESO (DOB) dynamics (uses measurement x1)
        hatx1dot = hatx2 + ell1*(x1 - hatx1);
        hatx2dot = hattheta * f + u + hatd + ell2*(x1 - hatx1);
        hatddot  = ell3*(x1 - hatx1);

        % adaptation with simple projection to [theta_min, theta_max]
        theta_dot = gamma * f * z2;
        if ( (hattheta >= par.theta_max && theta_dot>0) || (hattheta <= par.theta_min && theta_dot<0) )
            theta_dot = 0;
        end

        xdot = [x1dot; x2dot; hatx1dot; hatx2dot; hatddot; theta_dot];
    end

    % Smooth-ish saturation helper
    function y = saturate(s)
        y = max(-1,min(1,s));
    end
end
