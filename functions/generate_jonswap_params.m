function generate_jonswap_params(Hs, Tp, h, thetaSpreadDeg,g, gamma,N,maxVel,filename)
% Hs =  Significant wave height [m]
% Tp = Peak period [s]
% g = Gravity [m/s^2]
% gamma =  JONSWAP peak enhancement factor
% N =  Number of frequency components
% h =  Water depth [m]
% maxVel = max permissible velocity of the ocean current;
% thetaSpreadDeg = Small thetaSpreadDeg (e.g., 5°): Waves are mostly aligned, very directional sea. 
% % Larger thetaSpreadDeg (e.g., 30° or more): Waves come from a wider range of directions, representing more chaotic or multi-directional seas.
%filename: bname of file needs to be saved must end with .mat

    rng(42);                             
    f = linspace(0.05, 2, N);             
    df = f(2) - f(1);
    omega = 2 * pi * f;

    fp = 1 / Tp;
    sigma = 0.07 * (f <= fp) + 0.09 * (f > fp);
    r = exp(-(f - fp).^2 ./ (2 * sigma.^2 * fp^2));
    alpha = 0.076 * (g^2 / (2 * pi)^4) * Hs^2 / Tp^4;
    S = alpha .* f.^(-5) .* exp(-1.25 * (fp ./ f).^4) .* gamma.^r;

    a = sqrt(2 * S * df);

    % Scale amplitudes so max velocity < 0.6 m/s (surface, horizontal only)
    u_max_est = max(a .* omega);
    if u_max_est > maxVel
        scale_factor = maxVel / u_max_est;
        a = a * scale_factor;
    end

    k = omega.^2 / g;

    phi = 2 * pi * rand(1, N);


    % theta is directional spreading, save for velocity calculation
    thetaSpread = deg2rad(thetaSpreadDeg);
    theta = -thetaSpread + 2 * thetaSpread * rand(1, N);
    proj = matlab.project.rootProject;
    rootDir = proj.RootFolder;

    targetDir = fullfile(rootDir, 'Data');  % You can use any subfolder name
    if ~exist(targetDir, 'dir')
       mkdir(targetDir);
    end

    filePath = fullfile(targetDir, filename);  % .mat file path

    % Save parameters to struct or .mat file for use in velocity calc
    save(filePath, 'a', 'omega', 'k', 'phi', 'h', 'theta');

    % return parameters needed for velocity calculation
end
