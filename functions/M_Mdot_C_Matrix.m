function [M , M_dot, C] = M_Mdot_C_Matrix(rho, radius, length,l_dot, r_vec,r_dot, theta,omega,vlin)
% Calculate the 6x6 added mass matrix of a cylinder (general case)
%
% Inputs:
%   rho          - Fluid density
%   radius       - Cylinder radius [m]
%   length       - Cylinder length [m] 
%   l_dot        - rate of change of cylinder length
%   r_vec        - 3x1 vector from reference point (body COM) to cylinder COM [m]
%   r_dot        - 3x1 vector d(r_vec)/dt
%   Theta        - Angle of z axis roatation to make axis aline in radian (cylinder local -> body frame)
%   omega        - 3x1 vector of angular velocity  in body frame
%   vlin         - 3x1 vector of linear velocity in body frame 
% Output:
%   M      - 6x6 mass matrix in body frame 
%   M_dot  - 6x6 m_dot matrix in body frame 
%  C       - 6x6 coriolis matrix in body frame 



%% checks
P = inputParser;

addRequired(P,'r_dot',@(x) isnumeric(x) && iscolumn(x) && numel(x)==3)
addRequired(P,'r_vec',@(x) isnumeric(x) && iscolumn(x) && numel(x)==3)
addRequired(P,'omega',@(x) isnumeric(x) && isvector(x) && numel(x)==3)
addRequired(P,'vlin',@(x) isnumeric(x) && isvector(x) && numel(x)==3)



parse(P,r_dot,r_vec,omega,vlin)
r_vec = P.Results.r_vec;
r_dot = P.Results.r_dot;
omega = P.Results.omega;
vlin = P.Results.vlin;




% Volume of cylinder
V = pi * radius^2 * length;

% theta = deg2rad(theta);
R = [cos(theta) -sin(theta) 0;
    sin(theta) cos(theta) 0;
    0 0 1];

% Displaced fluid mass
m_disp = rho * V;

% Linear added mass matrix (local frame)
% Axial direction: x-axis of cylinder
% Transverse directions: y,z axes
M11_local = eye(3) * m_disp;

% Added inertia of fluid about cylinder COM in local frame
% Use standard moments of inertia formulas for a solid cylinder filled with fluid mass (added mass)
% Here, assuming fluid added mass distributed as cylinder mass with coefficients
% For fluid added inertia, multiply by coefficients as well

% Moments of inertia of solid cylinder about its center along principal axes
I_xx = 0.5 * m_disp * radius^2;                   % about cylinder axis (axial)
I_yy = (1/12) * m_disp * (3*radius^2 + length^2); % about transverse axis y
I_zz = I_yy;                                      % about transverse axis z

% Multiply moments by added mass coefficients
I_g_local = diag([I_xx, I_yy, I_zz]);

% Construct cross product matrix of r_vec
S_r = skmatrix(r_vec);

% Cross coupling matrices
M12 = m_disp * S_r;
M21 = -M12;

% Rotate inertia to body frame
I_new = R * I_g_local * R';

% Apply parallel axis theorem to inertia matrix about reference point
I_b = I_new + m_disp * (S_r * S_r');

% Rotate linear added mass matrix to body frame as well
M11 = R * M11_local * R';

% Assemble full 6x6 added mass matrix
M = [M11, M12;
     M21, I_b];



%% calculation of M dot

m1dot = rho*pi * radius^2 *l_dot;
M11dot_local = m1dot*eye(3);
M11dot = R*M11dot_local*R';
M12dot = -m1dot*skmatrix(r_vec)-m_disp*skmatrix(r_dot);
M21dot = -M12;

Ixx_dot = 0.5*rho*pi*radius^2*l_dot;
Iyy_dot = 1/12*(m1dot*(3*(r_vec'*r_vec)+length*length)+m_disp*(2*l_dot));
Izz_dot = Iyy_dot;

Idot_global = diag([Ixx_dot, Iyy_dot, Izz_dot]);


Idot = R*Idot_global*R';

M22dot = Idot + m1dot*(r_vec'*r_vec*eye(3)-r_vec*r_vec')+m_disp*(2*r_vec'*r_dot*eye(3) - (r_dot*r_vec'+r_vec*r_dot'));



M_dot = [M11dot M12dot; M21dot M22dot];


%% calcu lation of C


S_om = skmatrix(omega);
S_v = skmatrix(vlin);

adT = [S_om -S_v; zeros(3) S_om];

C = adT*M+ M_dot;


end