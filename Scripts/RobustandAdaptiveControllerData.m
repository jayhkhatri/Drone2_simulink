%% all the datas are in SI unit

m = 8.09; % fixed mass
m1 =1;
IB = diag([0.08 0.179 0.225]);
rho = 1000; %kg/m3
Rsyringe = 0.02; % m
g = 9.81; %m/s2
ZS = [0.0273; 0.0273; 0.0273]; % m 
l = 0.45;  % m
V = 8.2e-3; % m3
RG = [-0.002;0;-0.005]; % distance of cg from CO
rb = [0; 0 ; 0.008]; %m volume center
MaxOceanCurrent = 1 ; %m/s
Matfile = load("jonswap_params10.mat");
desired = [-2.5 0 0 0 0 0];

%%  gain constant and unknown parameter


k1e1= 1; k1e2 = 1;
parameter.K1e = diag([k1e1 k1e2]); %2x2

k2e1 = 10; k2e2 = 10; k2e3 =10;
parameter.K2eta1 = diag([k2e1 k2e2 k2e3]); %3x3

k3e1 = 5; k3e2 = 6;
parameter.K3eta2 = diag([k3e1 k3e2]);

d1 = 5; d2 =5;
parameter.Del = diag([d1 d2]); %2x2

g1 = 5; g2 = 5; g3 = 5;
parameter.Gama = diag([g1 g2 g3]);

parameter.sigma = 6;

parameter.Dtrue = diag([0.04 0.08 0.05]);
parameter.Ttrue = diag([0.002 0.002]);

parameter.opts.MaxIter = 10;
parameter.opts.lambdaLM0 = 1e-6;
parameter.opts.tol_r = 1e-6;
parameter.opts.tol_beta = 1e-6;
parameter.opts.regPi = 1e-6;


%% function data
Asx = pi*Rsyringe*Rsyringe*1.2;
Asz = 2*Rsyringe*l*1.2;
Asp = Asz;
funcfx = @(x) [0.5*Asx*rho*norm([x(1) x(2)])*x(1);0.5*Asz*rho*norm([x(1) x(2)])*x(2);0.5*Asp*rho*x(3)*x(3)*l];

%% defining main function.

f = @(t,x) statespace_modelZP(t,x,m,m1,IB,rho,g,Rsyringe,ZS,l,V,rb,RG,funcfx,MaxOceanCurrent,desired,MATfile,parameters);

%% function calling data
tspan = [0 10];
dt = 0.001;
x0 = zeros(27,1);



%% calling the function


[t,x] = ode23t(@(t,x) statespace_modelZP(t,x,m,m1,IB,rho,g,Rsyringe,ZS,l,V,rb,RG,funcfx,dt,MaxOceanCurrent,desired,Matfile,parameter),tspan,x0);
% [t,x] = rk4_fixed(@(t,x) statespace_modelZP(t,x,m,m1,IB,rho,g,Rsyringe,ZS,l,V,rb,RG,funcfx,dt,MaxOceanCurrent,desired,Matfile,parameter),tspan,x0,dt);


%%
% use your actual x0 and parameters used for ode45 call:

dx0 = statespace_modelZP(0, x0, m, m1, IB, rho, g, Rsyringe, ZS, l, V, rb, RG, funcfx, dt, MaxOceanCurrent, desired, Matfile, parameter);
whos dx0
any(isnan(dx0)), any(isinf(dx0))
