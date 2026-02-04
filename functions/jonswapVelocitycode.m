function [u, v, w]  = jonswapVelocitycode(t, z,maxVel,MATFile)
    persistent a omega k phi h theta
    if isempty(a)
        %params = Environment.OceanCurrentFile;
        a = MATFile.a;
        omega = MATFile.omega;
        k = MATFile.k;
        phi = MATFile.phi;
        h = MATFile.h;
        theta = MATFile.theta;
    end

    u = 0;
    v = 0;
    w = 0;

    for i = 1:length(a)
        wi = omega(i);
        ki = k(i);
        phi_i = phi(i);
        th = theta(i);
        denominator = sinh(k(i)*h);
        if abs(denominator) < 1e-8
           denominator = sign(denominator)*1e-8; % avoid zero division
        end

        scale_h = a(i) * wi * cosh(ki * (z + h)) / denominator;
        scale_v = a(i) * wi * sinh(ki * (z + h)) / denominator;

        phase = -wi * t + phi_i;

        u = u + scale_h * cos(th) * cos(phase);
        v = v + scale_h * sin(th) * cos(phase);
        w = w + scale_v * sin(phase);
    end

    % Optional: limit velocity magnitude here if needed
    mag = sqrt(u.^2 + v.^2 + w.^2);
    if mag > maxVel
        scale = maxVel / mag;
        u = u * scale;
        v = v * scale;
        w = w * scale;
    end
    %V_rel = [u;v;w];
end
