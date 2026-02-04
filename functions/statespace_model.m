function dx = statespace_model(~,x,m,m1,rho,g,AS,YS,ZS,l,RG)
%%%% defination



%% reuirement Checks and Pre-assign values
p = inputParser;
addRequired(p,'YS',@(x) isnumeric(x) && isvector(x) && numel(x)==3)
addRequired(p,'ZS',@(x) isnumeric(x) && isscalar(x))


parse(p,ZS,YS)
y1 = p.Results.YS(1); y2 = p.Results.YS(2); y3 = p.Results.YS(3);
z1 = p.Results.ZS;


%% state Identification
X =x(1);
Y = x(2);
Z = x(3);
r = x(4);
p = x(5);
y = x(6);
pos = x(1:3);
ori = x(3:6);
vel = x(7:12);
lf = x(13);
lr = x(14);
%% Mass Matrix
M = m+3*m1+rho*AS*(2*lf+lr);
M11= M*eye(3);
rgx = (m*RG(1)+m1*(y1+y2+y3)+rho*AS*lf(l-lf)-0.5*rho*AS*lr(l-lr))/M;
rgy = (m*RG(2)+m1*(y1+y2+y3)+2*rho*AS*lf*(y1+y2)+rho*AS*lr*y3)/M;
rgz = (m*RG(3)+(3*m1+rho*AS*(2*lf+lr))*z1)/M;

rg = [rgx; rgy; rgz];  %% location of CG from base frame

M12 = -M*skmatrix(rg);
M21 = -M12;
M22 = Ib;

M = [M11 M12; M21 M22];

%% Making Transformation matrix

Rbw = [cos(p)*cos(y) sin(r)*sin(p)*cos(y)-cos(r)*sin(y)  cos(r)*sin(p)*cos(y)+sin(r)*sin(y);
    cos(p)*sin(y) sin(r)*sin(p)*sin(y)+cos(r)*cos(y)  cos(r)*sin(p)*sin(y)-sin(r)*cos(y);
    -sin(p) sin(r)*cos(p) cos(r)*cos(p)];

Tbw = [1 sin(r)*tan(p) cos(r)*tan(p); 0 cos(r) -sin(r); 0 sin(r)/cos(p) cos(r)/cos(p)];

J = [Rbw zeros(3); zeros(3) Tbw];
%%









dx = [J*vel; 
    M\(u - C*vel-G-D*fx +d);
    T\(-l+uc)];
