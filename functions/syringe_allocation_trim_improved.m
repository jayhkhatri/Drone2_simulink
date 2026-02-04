function [Vf_cmd, Vb_cmd, Vf_trim_hat, Vb_trim_hat] = syringe_allocation_trim_improved(F_b, ex, edot_raw, Vf_prev, Vb_prev, ...
                                     Vf_trim_prev, Vb_trim_prev, dt, params)
% Keeps your dSum_trim = 0.25*Kp*ex/(1+Kr*abs(edot)) form but robustified.
% Units: meters (m). dt is allocator timestep (s). params is optional struct.

% -------- default params ----------
if nargin < 9 || isempty(params)
    params.Kp_trim = 0.02;       % your Kp used in trim (try 0.01-0.05)
    params.Kr = 0.08;            % suppression on rate (0.02-0.2)
    params.ex_db = 0.01;         % deadband in meters (1 cm)
    params.V_MIN = 0.005;        % m
    params.V_MAX = 0.100;        % m
    params.V_DOT_MAX = 0.012;    % m/s (12 mm/s)
    params.edot_lpf_fc = 1.0;    % Hz  (LPF cutoff for edot)
    params.tol_rate = 1e-6;      % small tol to avoid micro-steps
    params.leak_sigma = 0.0;     % optional small leakage (1e-5..1e-3)
    params.alloc_rate = 20;      % recommended allocator freq (Hz)
end

% ensure persistent LPF state
persistent edot_f_prev inited VF_TRIM VB_TRIM
if isempty(inited)
    edot_f_prev = 0;
    if isempty(Vf_trim_prev) || isempty(Vb_trim_prev)
        VF_TRIM = 0.046;  % init per-front trim (m)
        VB_TRIM = 0.031;  % init back trim (m)
    else
        VF_TRIM = Vf_trim_prev;
        VB_TRIM = Vb_trim_prev;
    end
    inited = true;
end

% ---------- 1) filter edot ----------
fc = params.edot_lpf_fc;
alpha = dt * 2*pi*fc / (1 + dt * 2*pi*fc);
edot_f = (1-alpha)*edot_f_prev + alpha*edot_raw;
edot_f_prev = edot_f;

% ---------- 2) compute adapt gate ----------
% smoother suppression than 1/(1+Kr*|edot|)
adapt_gate = exp(-params.Kr * abs(edot_f));  

% ---------- 3) deadband scaling ----------
db_scale = 1.0;
if abs(ex) < params.ex_db
    db_scale = 0.2;   % slow adaptation within deadband
end

% ---------- 4) your dSum_trim law (per-second) ----------
% your original shape: dSum_trim = 0.25 * Kp * ex / (1 + Kr * abs(edot))
% keep same numerator but use adapt_gate & db_scale
dSum_trim_per_s = params.Kp_trim * ex * adapt_gate * db_scale;  % m/s (stroke-sum)

% optional leak (small negative drift to avoid runaway)
if params.leak_sigma > 0
    VF_TRIM = VF_TRIM * (1 - params.leak_sigma*dt);
    VB_TRIM = VB_TRIM * (1 - params.leak_sigma*dt);
end

% ---------- 5) integrate trims (split 0.25 / 0.5) ----------
Vf_trim_new = VF_TRIM + 0.25 * dSum_trim_per_s * dt;
Vb_trim_new = VB_TRIM + 0.50 * dSum_trim_per_s * dt;

% ---------- 6) projection to stroke bounds (prevents windup) ----------
Vf_trim_new = min(max(Vf_trim_new, params.V_MIN), params.V_MAX);
Vb_trim_new = min(max(Vb_trim_new, params.V_MIN), params.V_MAX);

% ---------- 7) combine with fast reactive allocation (optional) ----------
% keep the usual mapping from F_b to fast increments (if you use it)
rho = 1000; g = 9.81; A = 2.0e-4;  % set correct A for your syringe
dSum_from_Fb = F_b / (rho*g*A);   % meters stroke-sum: 2*Vf + Vb = dSum_from_Fb
Vf_fast = 0.25 * dSum_from_Fb;
Vb_fast = 0.50 * dSum_from_Fb;

Vf_target = Vf_trim_new + Vf_fast;
Vb_target = Vb_trim_new + Vb_fast;

% ---------- 8) bounds on targets ----------
Vf_target = min(max(Vf_target, params.V_MIN), params.V_MAX);
Vb_target = min(max(Vb_target, params.V_MIN), params.V_MAX);

% ---------- 9) tolerant rate-limit (avoid micro-steps) ----------
max_step = params.V_DOT_MAX * max(dt, 1e-6);
dvf = Vf_target - Vf_prev;
if abs(dvf) <= params.tol_rate
    Vf_cmd = Vf_target;
else
    if abs(dvf) <= max_step
        Vf_cmd = Vf_target;
    else
        Vf_cmd = Vf_prev + sign(dvf)*max_step;
    end
end

dvb = Vb_target - Vb_prev;
if abs(dvb) <= params.tol_rate
    Vb_cmd = Vb_target;
else
    if abs(dvb) <= max_step
        Vb_cmd = Vb_target;
    else
        Vb_cmd = Vb_prev + sign(dvb)*max_step;
    end
end

% ---------- 10) export trims (for logging) and persist ---------- 
Vf_trim_hat = Vf_trim_new;
Vb_trim_hat = Vb_trim_new;
VF_TRIM = Vf_trim_new;
VB_TRIM = Vb_trim_new;

end
