%% Ocean current velocity
%function [a, omega, k, phi, h, t, dt] = generate_jonswap_params(Hs, Tp, t_end, dt, h, thetaSpreadDeg,g, gamma,N,filename)
Hs = 0.4;         % Significant wave height [m]
Tp = 6;           % Peak period [s]
g = Environment.gravity;         % Gravity [m/s^2]
gamma = 3.3;      % JONSWAP peak enhancement factor
N = 100;          % Number of frequency components
h = 10;           % Water depth [m]
maxVel = Environment.MaxOceanCurrent;
thetaSpreadDeg = 15;  %Small thetaSpreadDeg (e.g., 5°): Waves are mostly aligned, very directional sea. 
% Larger thetaSpreadDeg (e.g., 30° or more): Waves come from a wider range of directions, representing more chaotic or multi-directional seas.

generate_jonswap_params(Hs, Tp, h, thetaSpreadDeg,g, gamma,N,maxVel,'jonswap_params100.mat');

% force Fd = -0.5*cd*A.v_rel.|v_rel|

clear Hs Tp g gamma N h t_end dt thetaSpreadDeg maxVel
%% Wind Current Velocity