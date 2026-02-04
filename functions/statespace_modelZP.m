function dx = statespace_modelZP(t,x,m,m1,IB,rho,g,Rsyringe,ZS,l,V,rb,RG,funcfx,dt,MaxOceanCurrent,desired,MATfile,parameters)
%%%% defination
% x = states [z,p,u,w,q,lf,lr,lf_dot,lr_dot]
%  m = fixed mass
% m1 = moving mass 
% IB = Inertia tensor of 3*3
% rho = water density
% g = gravity (value in m/s^2)
% AS = syringe area
% ZS = CG location of z axis in each syringe [z1 z2 z3]
% l = length of vehicle/drone
% V = volume of drone
% rb = location of center of buoyancy from body frame [rbx rby rbz];
% RG = inital location of CG from body frame[ rgx rgy rgz];
% desired = target depth and pitch angle and velocities [z z_dot pitch
% pitch_rate]
% MATfile = file of disturnace jonswap.mat
% Parameters:    1x1 structure
%             
% 
% 
% fixed mass




warning('on','MATLAB:singularMatrix');
warning('on','MATLAB:nearlySingularMatrix');
warning('on','MATLAB:illConditionedMatrix');
dbstop if warning MATLAB:singularMatrix
dbstop if warning MATLAB:nearlySingularMatrix
dbstop if warning MATLAB:illConditionedMatrix



%% reuirement Checks and Pre-assign values
P = inputParser;
%addRequired(p,'YS',@(x) isnumeric(x) && isvector(x) && numel(x)==3)
addRequired(P,'ZS',@(x) isnumeric(x) && iscolumn(x) && numel(x) ==3)
addRequired(P,'rb',@(x) isnumeric(x) && iscolumn(x) && numel(x)==3)
% addRequired(P,'IB', @(x) isnumeric(X) && ismatrix(x));
addRequired(P,'parameters',@(x) isstruct(x));
parse(P,ZS,rb,parameters)
%y1 = P.Results.YS(1); y2 = p.Results.YS(2); y3 = p.Results.YS(3);
ZS = P.Results.ZS;
rb = P.Results.rb;
% IB = P.Results.IB;
parameters = P.Results.parameters;

%% control gain constants

%<------- error constant -------->

Dtrue = parameters.Dtrue;
K1 = parameters.K1e;
K2 = parameters.K2eta1;
K3 = parameters.K3eta2;
Del = parameters.Del;
gama = parameters.Gama;
sigma = parameters.sigma;
Ttrue = parameters.Ttrue;
opts= parameters.opts;



%% extra
AS = pi*Rsyringe^2;
arc = [1 3 5];
jr = [3 5];
%% state Identification
[m11,n1] = size(Dtrue);
[m22,n22] = size(Ttrue);

Z = x(1);
r=0;y =0;
p = x(2);
% pos = x(1:3);
% ori = x(3:6);
u = x(3);
w = x(4);
q = x(5);
vel = [u;0;w];
omega = [0;q;0];
%vel = x(3:4);
lf = x(6);
lr = x(7);
Dcap_vec = x(8:16);
Dcap = reshape(Dcap_vec,m11,n1);
ud_prev = [x(17);x(18);x(19)];
ld_prev = [x(20);x(21)];
Tcap_vec= x(22:25);
Tcap = reshape(Tcap_vec,m22,n22);
lf_dot = x(26);
lr_dot = x(27);

ld_dot_prev= [lf_dot;lr_dot];
vel1 = [u;w;q];

%% Mass Matrix and coriollis matrix
Mdead = [m*eye(3) -m*skmatrix(RG); m*skmatrix(RG) IB ];  %% dead weight mass mo
Mdead11 = m*eye(3); Mdead12 = -m*skmatrix(RG); Mdead21=-Mdead12; Mdead22 = IB;
Cdead = [zeros(3)  -skmatrix(Mdead11*vel+Mdead12*omega);-skmatrix(Mdead11*vel+Mdead12*omega) -skmatrix(Mdead21*vel+Mdead22*omega)];

[MF1, MF1_dot ,CF1] = M_Mdot_C_Matrix(rho,Rsyringe,lf,lf_dot,[(l-lf)/2; 0 ; ZS(1)],[-lf_dot/2 ;0 ;0],pi,omega,vel);
[MF2, MF2_dot ,CF2] = M_Mdot_C_Matrix(rho,Rsyringe,lf,lf_dot,[(l-lf)/2; 0 ; ZS(2)],[-lf_dot/2 ;0 ;0],pi,omega,vel);
[MR, MR_dot ,CR] = M_Mdot_C_Matrix(rho,Rsyringe,lr,lr_dot,[(l-lr)/2; 0 ; ZS(3)],[-lr_dot/2 ;0 ;0],0,omega,vel);


% MF1 = added_mass_cylinder(rho,Rsyringe,lf,[0.5*(l-lf) 0 ZS(1)],pi,1,1);
% MF2 = added_mass_cylinder(rho,Rsyringe,lf,[0.5*(l-lf) 0 ZS(2)],pi,1,1);
% MR =  added_mass_cylinder(rho,Rsyringe,lr,[0.5*(l-lr) 0 ZS(3)],pi,1,1);


M = Mdead + MF1 +MF2 +MR;
C = Cdead+ CF1 +CF2+ CR;

M_dot = MF1_dot+MF2_dot+MR_dot;


% rgx = (m*RG(1)+m1*((3*l/2)-2*lf-lr)+rho*AS*lf(l-lf)-0.5*rho*AS*lr(l-lr))/M;
% rgy = (m*RG(2)+m1*(y1+y2+y3)+2*rho*AS*lf*(y1+y2)+rho*AS*lr*y3)/M;
% rgz = (m*RG(3)+(3*m1+rho*AS*(2*lf+lr))*z1)/M;
% 
% rg = [rgx; rgy; rgz];  %% location of CG from base frame



invM =  Mdead\eye(6); %(M+0.001*eye(6))\eye(6);

M = M(arc,arc);
M_dot = M_dot(arc,arc);
invM = invM(arc,arc);
C = C(arc,arc);



%% Making Transformation matrix

Rbw = [cos(p)*cos(y) sin(r)*sin(p)*cos(y)-cos(r)*sin(y)  cos(r)*sin(p)*cos(y)+sin(r)*sin(y);
    cos(p)*sin(y) sin(r)*sin(p)*sin(y)+cos(r)*cos(y)  cos(r)*sin(p)*sin(y)-sin(r)*cos(y);
    -sin(p) sin(r)*cos(p) cos(r)*cos(p)];

Tbw = [1 sin(r)*tan(p) cos(r)*tan(p); 0 cos(r) -sin(r); 0 sin(r)/cos(p) cos(r)/cos(p)];

J = [Rbw zeros(3); zeros(3) Tbw]; % 6*6 matrix

J = J(jr,arc); %2*3 matrix [-sin(p) cos(p) 0; 0 0 1]
invJ = pinv(J); % or simply j' as J*ji  = I2;      3*2 matrix 

Jdot = [-cos(p)*q -sin(p)*q 0; 0 0 0];

%% Gravity Matrix
Fgn = [0;0;-Mdead(1,1)*g];
Fbn = [0;0;rho*V*g];

Fgb = Rbw'*Fgn;
Fbb = Rbw'*Fbn;

G = -[Fgb+Fbb; cross(RG,Fgb)+cross(rb,Fbb)]; % 6*1 matrix
G = G(arc); % 3*1 vector

%% Disturbance term
[dx, dy, dz] = jonswapVelocitycode(t,Z,MaxOceanCurrent,MATfile);
D= Rbw'*[dx;dy;dz];
d= [D;0;0;0]; % 6*6 matrix
d= d(arc);  % 3*1 vector

%% desired 
zd = desired(1);
pd = desired(2);

eta_dot_d = [desired(3);desired(4)];
eta_ddot_d = [desired(5);desired(6)];


%% error term
e= [Z;p]-[zd; pd];



%% first virtual controller \vd


vd = invJ*(eta_dot_d -K1*e);

eta1 = [u;w;q] - vd;


e_dot = J*eta1-K1*e;

vd_dot = -invJ*Jdot*invJ*(eta_dot_d-K1*e)+invJ*(eta_ddot_d-K1*e_dot);

%% second virtual controller u

fx = funcfx(vel1);


ud_new = invM*(C*vel1+G + Dcap*fx - pinv(eta1')*e'*J*eta1-K2*eta1+M*vd_dot);  %% tau desired

ud_dot = (ud_new-ud_prev)/dt;

%% last controller uc
h_fun = @(zz,ll) [(MF1(1,1)+MF2(1,1)+MR(1,1))*g*sin(zz); 
    -(MF1(1,1)+MF2(1,1)+MR(1,1))*g*cos(zz); 
    (MF1(1,1)+MF2(1,1))*g*(cos(zz)*((l-ll(1))/2)+ZS(1)*sin(zz))+(MR(1,1))*g*(cos(zz)*(-(l-ll(2))/2)+ZS(3)*sin(zz))];

Jh_func = @(zz,ll) [rho*g*AS*2*sin(zz) rho*g*AS*sin(zz); -rho*g*2*cos(zz) -rho*g*AS*cos(zz); rho*g*AS*((l-2*ll(1))*cos(zz)+2*ZS(1)*sin(zz))  rho*g*AS*(0.5*(l-2*ll(2))*cos(zz)+ZS(3)*sin(zz)) ];



%                   .beta_min    (optional box constraint)
%                   .beta_max
% opts.MaxIter = 10;        %    .maxIter     (default 10)
% opts.lambdaLM0 = 1e-6;    %    .lambdaLM0   (default 1e-6)
% opts.tol_r = 1e-6;        %    .tol_r       (default 1e-6)
% opts.tol_beta = 1e-6;     %    .tol_beta    (default 1e-6)
% opts.regPI = 1e-6;        %    .regPI       (default 1e-6)


ld= solve_beta(h_fun,Jh_func,p,ud_new,[0.005; 0.005],opts);
ld = clip(ld,0.001,0.1);

Jl = Jh_func(p,ld);

invJl = pinv(Jl);


Jn = [(MF1(1,1)+MF2(1,1)+MR(1,1))*g*cos(p); (MF1(1,1)+MF2(1,1)+MR(1,1))*g*sin(p);  
    (MF1(1,1)+MF2(1,1))*g*(-sin(p)*((l-ld(1))/2)+ZS(1)*cos(p))+(MR(1,1))*g*(-sin(p)*(-(l-ld(2))/2)+ZS(3)*cos(p))];

ld_dot = invJl*(ud_dot-Jn*q);

l_vec = [lf;lr];
eta2 = l_vec-ld;

eta2 =reshape(eta2,2,1);
%% actual control
uc = -l_vec+Tcap*ld_dot-K3*eta2;


%% State space model function
Dcap_dot = -gama*eta1*fx'-sigma.*(gama*Dcap);
% Defensive checks (will throw informative error)
assert(all(size(Del) == [2,2]), 'Del must be 2x2');
assert(all(size(eta2) == [2,1]), 'eta2 must be 2x1');
% disp(['size INVM = ', mat2str(size(invM))]);
% disp(['size C = ', mat2str(size(C))]);
% disp(['size vel1 = ', mat2str(size(vel1))]);
% disp(['size G = ', mat2str(size(G))]);
% disp(['size dcap = ', mat2str(size(Dcap))]);
% disp(['size fx = ', mat2str(size(fx))]);
% disp(['size pinveta1 = ', mat2str(pinv(eta1'))]);
% disp(['size edash = ', mat2str(size(e'))]);
% disp(['size J = ', mat2str(size(J))]);
% disp(['size eta1 = ', mat2str(size(eta1))]);
% disp(['size K2 = ', mat2str(size(K2))]);
% disp(['size M = ', mat2str(size(M))]);
% disp(['size vd_dot = ', mat2str(size(vd_dot))]);

assert(all(size(ld_dot) == [2,1]), 'ld_dot must be 2x1');

Tcap_dot = -Del*eta2*transpose(ld_dot);

dx = [J*vel1; 
    invM*(ud_new - C*vel1-G-Dtrue*fx +d);
    Ttrue\(-l_vec+uc);
    Dcap_dot(:);
    ud_dot;
    ld_dot;
    Tcap_dot(:);
    (ld_dot-ld_dot_prev)/dt];
end