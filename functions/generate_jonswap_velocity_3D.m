function generate_jonswap_velocity_3D()
% Generates JONSWAP spectrum based 3D velocity and saves to jonswap_params.mat

rng(42); % For repeatability

% Parameters
Hs = 0.2;         % Significant wave height [m]
Tp = 6;           % Peak period [s]
g = 9.81;         % Gravity [m/s^2]
gamma = 3.3;      % JONSWAP peak enhancement factor
N = 100;          % Number of frequency components
h = 10;           % Water depth [m]
z = -1;           % Depth below surface [m]
t_end = 1000;       % Duration [s]
dt = 0.01;        % Time step [s]
t = 0:dt:t_end;

% Frequency domain
f_min = 0.05; f_max = 2.0; 
f = linspace(f_min, f_max, N);
df = f(2) - f(1);
omega = 2 * pi * f;
k = omega.^2 / g; % Linear wave theory

% JONSWAP spectrum
alpha = 0.076 * (Hs^2) / (Tp^4);
sigma = 0.07 * (f <= 1/Tp) + 0.09 * (f > 1/Tp);
r = exp(-( (f - 1/Tp).^2 ) ./ (2 * sigma.^2 * (1/Tp)^2));
S = alpha * g^2 ./ (omega.^5) .* exp(-1.25 * (f * Tp).^(-4)) .* gamma.^r;
a = sqrt(2 * S * df);
phi = 2 * pi * rand(1, N); % random phase

% Calculate velocities
u = zeros(size(t));
v = zeros(size(t));
w = zeros(size(t));

for i = 1:N
    omega_i = omega(i);
    k_i = k(i);
    a_i = a(i);
    phi_i = phi(i);
    
    scale_u = a_i * omega_i * cosh(k_i * (z + h)) / sinh(k_i * h);
    scale_w = a_i * omega_i * sinh(k_i * (z + h)) / sinh(k_i * h);
    
    u = u + scale_u * cos(-omega_i * t + phi_i);
    w = w + scale_w * sin(-omega_i * t + phi_i);
end

% Assume v is zero for now (2D wave propagation)
v = zeros(size(u));

% Normalize to ensure max magnitude <= 0.6 m/s
mag = sqrt(u.^2 + v.^2 + w.^2);
if max(mag) > 0.6
    scale = 0.6 / max(mag);
    u = u * scale;
    v = v * scale;
    w = w * scale;
    a = a * scale;
end

% Save parameters (for simulink runtime use)
save('jonswap_params.mat', 'a', 'omega', 'k', 'phi', 'h', 'z');
end