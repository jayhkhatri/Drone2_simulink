function generate_jonswap_params1(Hs, Tp, h, thetaSpreadDeg, g, gamma, N, filename)

% ---------------------------------------------------------
% Corrected JONSWAP Spectrum Generator
% Energy-consistent with prescribed Hs
% ---------------------------------------------------------

rng(42);   % Repeatability

% Frequency domain
f_min = 0.05;
f_max = 2.0;
f = linspace(f_min, f_max, N);
df = f(2) - f(1);
omega = 2*pi*f;
fp = 1/Tp;

% JONSWAP shape parameters
sigma = 0.07*(f <= fp) + 0.09*(f > fp);
r = exp(-(f - fp).^2 ./ (2*sigma.^2*fp^2));

% --- Proper JONSWAP formulation (frequency domain) ---
S = (g^2/(2*pi)^4) .* f.^(-5) ...
    .* exp(-1.25*(fp./f).^4) ...
    .* gamma.^r;

% ---------------------------------------------------------
% ENERGY NORMALIZATION (CRITICAL STEP)
% ---------------------------------------------------------
m0 = sum(S*df);
Hs_current = 4*sqrt(m0);

% Scale spectrum so that Hs matches exactly
scale_factor = (Hs / Hs_current)^2;
S = S * scale_factor;

% Recompute to confirm
m0 = sum(S*df);
Hs_check = 4*sqrt(m0);
fprintf('Target Hs = %.3f m | Generated Hs = %.3f m\n', Hs, Hs_check);

% Wave amplitudes
a = sqrt(2*S*df);

% Deep-water approximation (acceptable for Tp=6s, h=10m)
k = omega.^2 / g;

% Random phase
phi = 2*pi*rand(1,N);

% Directional spreading
thetaSpread = deg2rad(thetaSpreadDeg);
theta = -thetaSpread + 2*thetaSpread*rand(1,N);



proj = matlab.project.rootProject;
rootDir = proj.RootFolder;
targetDir = fullfile(rootDir, 'Data');  % You can use any subfolder name
    if ~exist(targetDir, 'dir')
       mkdir(targetDir);
    end
filePath = fullfile(targetDir, filename);

% Save clean parameters
save(filePath, 'a', 'omega', 'k', 'phi', 'h', 'theta');

fprintf('JONSWAP parameters saved to %s\n', filename);

end