%%%% data files for environment and vehicle data
%
%
%
%% Environment General Data
Environment.gravity = -9.81; %m/s^2
Environment.Airdensity = 1.013; %kg/m^3
Environment.Waterdensity = 1000; %kg/m^3
Environment.kinematicviscocity = 1.1923e-6;             % m/s^2 @15oC
Environment.sealevelpressure = 101325.0;                % N/m^2
Environment.SimulationTime = 150;
Environment.SimulationDt = 0.01;
Environment.MaxOceanCurrent = 4; % m/s
Environment.MaxAirSpeed = 6; %m/s
Environment.Transition = 0.1; 
Environment.AirTransitionSpeed = 0.8;
Environment.UnderwaterTransitionSpeed= 0.1;
Environment.AirTransitionAgularSpeed = 0.1;
Environment.UnderwaterTransitionAngularSpeed = 0.01;
%% simulation data

Environment.Simulation.Runtime = 500;
Environment.Simulation.GlobalSteptime = 0.001; % must be in multiple of local
Environment.Simulation.LocatSteptime = 0.001; % must be lower or same as global

%% vehicle data
% Vehicle initial condition
Vehicle.InitCond.position = [0 0 -2.5];                    % m
Vehicle.InitCond.velocity = [0 0 0];                    % m/s
Vehicle.InitCond.EulerXYZ = [0 0 0];                    % deg [roll,pitch,yaw]
Vehicle.InitCond.rotates = [0 0 0];                    % rad/s

% Convert Euler angles to Radians and compute rotation matrix
Vehicle.InitCond.EulerXYZrad = Vehicle.InitCond.EulerXYZ*pi/180;
Vehicle.InitCond.RotMZYX = eul2rotm(Vehicle.InitCond.EulerXYZrad,'ZYX');



%% Vehicle Body
Vehicle.Body.Length =0.450; %m
Vehicle.Body.Hulldiameter = 0.170; %m
Vehicle.Body.XYArea = 0.3290; % m^2 top plan
Vehicle.Body.XZArea = 0.25; % m^2 side view
Vehicle.Body.YZArea = 0.050; %m^2 front view
Vehicle.Body.Volumn = 0.0108442; % m^3
%% Buoyancy force:
%Vehicle.MaxBuoyancyForce = abs(round(Vehicle.Body.Volumn*Environment.Waterdensity*Environment.gravity,3)); %N 
Vehicle.MaxBuoyancyForce =80.858; %80.653; % -7.75*Environment.gravity;

%% Vehicle additional data regrading CG
Vehicle.Dist.CGtoBottom = 0.082; % m
Vehicle.Dist.CGtoTop = round(Vehicle.Body.Hulldiameter-Vehicle.Dist.CGtoBottom,4); % m

%% motor and thruster parameters
Vehicle.Motor.Stiffness = 10; % N.m/degree
Vehicle.Motor.Damping = 200; % N.m/(degree/sec)
Vehicle.Motor.Maxtorque = 1.75; %N.m
Vehicle.Motor.MaxThrust = 45 ; %N
Vehicle.Thruster.Stiffness = 10; % N.m/degree
Vehicle.Thruster.Damping = 200; % N.m/(degree/sec)
Vehicle.Thruster.Maxtorque = 1; %N.m
Vehicle.Thruster.MaxForwardThrust = 60; %N
Vehicle.Thruster.MaxReverseThrust = 55; %N

%% syringe motion parameters
Vehicle.Syringe.Damping = 50; % N/(m/s)  20 ideal
Vehicle.Syringe.Stiffness = 500; % N/m
Vehicle.Syringe.Front.minpos = 0.005; %m
Vehicle.Syringe.Front.maxpos = 0.1; %m
Vehicle.Syringe.Front.Optpos = 0.046;%0.045;%0.039805; %m 0.031
Vehicle.Syringe.Front.Equipose = Vehicle.Syringe.Front.maxpos;
Vehicle.Syringe.Front.Initpose = Vehicle.Syringe.Front.maxpos;
Vehicle.Syringe.Back.minpos = 0.005; %m
Vehicle.Syringe.Back.maxpos = 0.1; %m
Vehicle.Syringe.Back.Optpos = 0.031;%0.041; %m 0.0516
Vehicle.Syringe.Back.Initpose = Vehicle.Syringe.Back.maxpos;
Vehicle.Syringe.Back.Equipose = Vehicle.Syringe.Back.maxpos;
Vehicle.Syringe.Thrustlim = 50; %N
Vehicle.Syringe.Radius = 0.029; %m
Vehicle.Syringe.MaxVelocity = 0.012;
% hower.front: 0.033
% hower.Back = 0.036
% for forward thrust or rotation or any locomotion involving thrust between -1 to 1 idea pos of syringe are 0.057 and 0.012
% respectively for no added mass. this values are suitable to any thrust
% value with sum of total thrust (LT+RT) is between 0 to 2. for the
% locamotion in negative direction or in the event of both negative thrust
% this value doesnt hold.


%% main body  cylinderical
Vehicle.AddedMass.Body.Radius = 0.235;
Vehicle.AddedMass.Body.Length = 0.5;
Vehicle.AddedMass.Body.r = [0 0 0]; 
Vehicle.AddedMass.Body.C_axial = 0.5;
Vehicle.AddedMass.Body.C_transverse =0.8;

Vehicle.AddedMass.M_added_Body = added_mass_cylinder(Environment.Waterdensity,Vehicle.AddedMass.Body.Radius, ...
    Vehicle.AddedMass.Body.Length,Vehicle.AddedMass.Body.r,0,Vehicle.AddedMass.Body.C_axial,Vehicle.AddedMass.Body.C_transverse);

%% For rotors Sphere
Vehicle.AddedMass.Sphere(1).Radius = 0.03;
Vehicle.AddedMass.Sphere(2).Radius = 0.03;
Vehicle.AddedMass.Sphere(3).Radius = 0.03;
Vehicle.AddedMass.Sphere(4).Radius = 0.03;
Vehicle.AddedMass.Sphere(1).R = [0.155 -0.276 0.08475];
Vehicle.AddedMass.Sphere(2).R = [0.155 0.276 0.08475];
Vehicle.AddedMass.Sphere(3).R = [-0.185 0.276 0.08475];
Vehicle.AddedMass.Sphere(4).R = [-0.185 -0.276 0.08475];


Vehicle.AddedMass.M_Added_sphere = zeros(6);
for i=1:length(Vehicle.AddedMass.Sphere)
    Added_mass = added_mass_sphere(Vehicle.AddedMass.Sphere(i).Radius,Environment.Waterdensity,Vehicle.AddedMass.Sphere(i).R);
    Vehicle.AddedMass.M_Added_sphere = Vehicle.AddedMass.M_Added_sphere + Added_mass;
end
clear Added_mass i


%% for Thruster Cylinderical
Vehicle.AddedMass.Thruster(1).Radius = 0.05;
Vehicle.AddedMass.Thruster(2).Radius = 0.05;
Vehicle.AddedMass.Thruster(1).Length = 0.05;
Vehicle.AddedMass.Thruster(2).Length = 0.05;

Vehicle.AddedMass.Thruster(1).R = [-0.031 -0.154 -0.003];
Vehicle.AddedMass.Thruster(2).R = [-0.031  0.154 -0.003];

Vehicle.AddedMass.Thruster(1).C_axial = 0.25;
Vehicle.AddedMass.Thruster(1).C_transverse= 0.7;
Vehicle.AddedMass.Thruster(2).C_axial = 0.25;
Vehicle.AddedMass.Thruster(2).C_transverse= 0.7;

Vehicle.AddedMass.M_added_Thruster = zeros(6);
for i = 1:length(Vehicle.AddedMass.Thruster)
    Added_mass = added_mass_cylinder(Environment.Waterdensity,Vehicle.AddedMass.Thruster(i).Radius, ...
        Vehicle.AddedMass.Thruster(i).Length,Vehicle.AddedMass.Thruster(i).R,0, ...
        Vehicle.AddedMass.Thruster(i).C_axial,Vehicle.AddedMass.Thruster(i).C_transverse);
    
    Vehicle.AddedMass.M_added_Thruster = Vehicle.AddedMass.M_added_Thruster + Added_mass;
end
clear i Added_mass

%% for arms cylinder with angle in cordinates
Vehicle.AddedMass.Arm(1).Radius = 0.021;
Vehicle.AddedMass.Arm(2).Radius = 0.021;
Vehicle.AddedMass.Arm(3).Radius = 0.0321;
Vehicle.AddedMass.Arm(4).Radius = 0.021;


Vehicle.AddedMass.Arm(1).Length = 0.14;
Vehicle.AddedMass.Arm(2).Length = 0.14;
Vehicle.AddedMass.Arm(3).Length = 0.14;
Vehicle.AddedMass.Arm(4).Length = 0.14;


Vehicle.AddedMass.Arm(1).R = [0.155 -0.170 0.070];
Vehicle.AddedMass.Arm(2).R = [0.155 0.170 0.070];
Vehicle.AddedMass.Arm(3).R = [-0.185 0.170 0.070];
Vehicle.AddedMass.Arm(4).R = [-0.185 -0.170 0.070];


Vehicle.AddedMass.Arm(1).C_axial = 0.1;
Vehicle.AddedMass.Arm(2).C_axial = 0.1;
Vehicle.AddedMass.Arm(3).C_axial = 0.1;
Vehicle.AddedMass.Arm(4).C_axial = 0.1;

Vehicle.AddedMass.Arm(1).C_transverse = 0.9;
Vehicle.AddedMass.Arm(2).C_transverse = 0.9;
Vehicle.AddedMass.Arm(3).C_transverse = 0.9;
Vehicle.AddedMass.Arm(4).C_transverse = 0.9;

% theta angle of arm with respect to x axis in degree between -pi/2 to pi/2
% to make it aline the axis of cylinder to x axis
Vehicle.AddedMass.Arm(1).theta = 90;
Vehicle.AddedMass.Arm(2).theta = -90;
Vehicle.AddedMass.Arm(3).theta = 90;
Vehicle.AddedMass.Arm(4).theta = -90;

Vehicle.AddedMass.M_added_Arm = zeros(6);
for i=1:length(Vehicle.AddedMass.Arm)
    Added_mass = added_mass_cylinder(Environment.Waterdensity,Vehicle.AddedMass.Arm(i).Radius,Vehicle.AddedMass.Arm(i).Length, ...
        Vehicle.AddedMass.Arm(i).R,Vehicle.AddedMass.Arm(i).theta, ...
        Vehicle.AddedMass.Arm(i).C_axial,Vehicle.AddedMass.Arm(i).C_transverse);
    Vehicle.AddedMass.M_added_Arm = Vehicle.AddedMass.M_added_Arm + Added_mass;

end
clear i Added_mass
%% Added mass due to landing gear



%% added mass parameters
Vehicle.AddedMassMatrix = Vehicle.AddedMass.M_added_Body+Vehicle.AddedMass.M_Added_sphere +Vehicle.AddedMass.M_added_Thruster+Vehicle.AddedMass.M_added_Arm;
%Vehicle.AddedMass = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0 ;0 0 0 0 1 0;0 0 0 0 0 1];



%% coefficient of resistance in air and water.
Vehicle.Air.Resistance.Cx = 0.00088;
Vehicle.Air.Resistance.Cy = 0.00418;
Vehicle.Air.Resistance.Cz = 0.00237;
Vehicle.Air.Resistance.Ck = 0.001125;
Vehicle.Air.Resistance.Cm = 0.000471;
Vehicle.Air.Resistance.Cn = 0.000376;

Vehicle.Water.Resistance.Cx = 0.8;
Vehicle.Water.Resistance.Cy = 2.5;
Vehicle.Water.Resistance.Cz = 1;
Vehicle.Water.Resistance.Ck = 0.8;
Vehicle.Water.Resistance.Cm = 10;%1.2
Vehicle.Water.Resistance.Cn = 1.8;

Vehicle.Air.LinearResistance.Cx = 0.002;
Vehicle.Air.LinearResistance.Cy = 0.001418;
Vehicle.Air.LinearResistance.Cz = 0.001837;
Vehicle.Air.LinearResistance.Ck = 0.1125;
Vehicle.Air.LinearResistance.Cm = 0.471;
Vehicle.Air.LinearResistance.Cn = 0.00876;

Vehicle.Water.LinearResistance.Cx = 4.0;
Vehicle.Water.LinearResistance.Cy = 20.0;
Vehicle.Water.LinearResistance.Cz = 20.0;
Vehicle.Water.LinearResistance.Ck = 6.5; %650;
Vehicle.Water.LinearResistance.Cm = 1.2; %1200000;
Vehicle.Water.LinearResistance.Cn = 1.5; %150;

%% ocean curent data
Environment.Ocean.cdx = 1;  % between 0.8 to 1.2
Environment.Ocean.cdy = 1;
Environment.Ocean.cdz = 1;
Environment.OceanCurrentFile = load('jonswap_params100.mat');
A = Bus_element(Environment.OceanCurrentFile);

%% Wind Disturbance
Environment.Wind.mean = [1 0.5 0];
Environment.Wind.MAX = 6;
Environment.Wind.cdx = 0.08;
Environment.Wind.cdy = 0.08;
Environment.Wind.cdz = 0.1;


%% taget data
Vehicle.Target.z = -5;
Vehicle.Target.pitch =0;
Vehicle.Target.v.w = 0;
Vehicle.Target.dwbdt.pitchrate = 0;
Vehicle.Target.v.aw = 0;
Vehicle.Target.dwbdt.pitchangacc = 0;

Vehicle.Target.Desired = [Vehicle.Target.z Vehicle.Target.pitch Vehicle.Target.v.w Vehicle.Target.dwbdt.pitchrate Vehicle.Target.v.aw Vehicle.Target.dwbdt.pitchangacc]';


Proj.Name = matlab.project.rootProject;
Proj.rootDir = Proj.Name.RootFolder;
Proj.stldir = fullfile(Proj.rootDir,'stl');