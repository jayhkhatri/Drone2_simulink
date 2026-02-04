function M = added_mass_sphere(R, density, r)
% Calculate 6x6 added mass matrix of a sphere from geometry and offset
% radius  : radius of the sphere (m)
% density : fluid density (kg/m^3)
% r       : 3x1 vector from body COM to sphere COM (in body frame)

% Validate input
if numel(r) ~= 3
    error('Offset vector r must be a 3x1 vector');
end

% Compute added mass for sphere (same in x, y, z)
m = (2/3) * pi * density * R^3;


% Added inertia about sphere COM (diagonal)
Ig = (2/5) * m * R^2 * eye(3);


% Skew-symmetric matrix of offset
S = skmatrix(r);

% 6x6 added mass matrix
M = [ m * eye(3),     m * S;
     -m * S,     Ig- m * (S * S)];
end
