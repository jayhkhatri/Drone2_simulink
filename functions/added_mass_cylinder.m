function M_added = added_mass_cylinder(rho, radius, length, r_vec, theta, C_axial, C_transverse)
% Calculate the 6x6 added mass matrix of a cylinder (general case)
%
% Inputs:
%   rho          - Fluid density
%   radius       - Cylinder radius [m]
%   length       - Cylinder length [m]
%   r_vec        - 3x1 vector from reference point (body COM) to cylinder COM [m]
%   R            - 3x3 rotation matrix (cylinder local -> body frame)
%   C_axial      - Added mass coefficient in axial direction (x axis of cylinder)
%   C_transverse - Added mass coefficient in transverse directions (y,z axes)
%
% Output:
%   M_added      - 6x6 added mass matrix in body frame

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
M11_local = diag([C_axial, C_transverse, C_transverse]) * m_disp;

% Added inertia of fluid about cylinder COM in local frame
% Use standard moments of inertia formulas for a solid cylinder filled with fluid mass (added mass)
% Here, assuming fluid added mass distributed as cylinder mass with coefficients
% For fluid added inertia, multiply by coefficients as well

% Moments of inertia of solid cylinder about its center along principal axes
I_xx = 0.5 * m_disp * radius^2;                   % about cylinder axis (axial)
I_yy = (1/12) * m_disp * (3*radius^2 + length^2); % about transverse axis y
I_zz = I_yy;                                      % about transverse axis z

% Multiply moments by added mass coefficients
I_g_local = diag([I_xx*C_axial, I_yy*C_transverse, I_zz*C_transverse]);

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
M_added = [M11, M12;
           M21, I_b];
end

