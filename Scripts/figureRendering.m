global basepath FIG_SAVE_PATH;

proj = currentProject;
path = proj.RootFolder;
basepath = fullfile(proj.RootFolder,'Results','fig','2026','April','20042026');
FIG_SAVE_PATH = fullfile(proj.RootFolder,'Results','epspdf','2026','April','20042026');
MATfilePath = fullfile(proj.RootFolder,"Results","MAT");
%% Savind Data in .MAT file

if ~exist(MATfilePath, 'dir')
    mkdir(MATfilePath);
end

filename = sprintf('results_SimpleBSwithoutSTA_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));

save(fullfile(MATfilePath, filename), 'out');

%% to load file   %% use only when need to run data from .MAT file rather than simuluink

file = fullfile(MATfilePath,"results_20260325_132143.mat");
AA = load(file);


% out = AA.out;  % use this when need to draw graph from mat file and not
% from the simulink out put.

%% 
t = out.tout;
Xm = out.data.COM.World.Xe.X.Data;
Ym = out.data.COM.World.Xe.Y.Data;
Zm  = out.data.COM.World.Xe.Z.Data;
roll = out.data.COM.Derived.Euler.phi.Data;
pitch = out.data.COM.Derived.Euler.theta.Data;
yaw = out.data.COM.Derived.Euler.psi.Data;

%% in degree% %
rolld = rad2deg(roll);
pitchd = rad2deg(pitch);
yawd = rad2deg(yaw);
%% to take data from error
edata = squeeze(out.error.Data);  % CONVERTS 3D ARRAY TO 2D BY REMOVING TIME
Ez = edata(1,:)';
Epitch = edata(2,:)';

%% to take data from force
fdata = squeeze(out.force.Data);  % CONVERTS 3D ARRAY TO 2D BY REMOVING TIME
u1 = fdata(1,:)';
u2 = fdata(2,:)';

%% syringe data

s1l = out.data.Buoyancy.Syringe.SyringeFL.Pose.Data;
s1r = out.data.Buoyancy.Syringe.SyringeFR.Pose.Data;
s2 = out.data.Buoyancy.Syringe.SyringeB.Pose.Data;


%% plotiing function

%% plotting error plot
N = 1201; % sample points
figure('Units','normalized','OuterPosition',[0 0 1 1])
% rectangle('Position',[0 -0.15 1200 0.3],'FaceColor',"g",FaceAlpha=0.1,EdgeColor=[1 1 1]);
% yyaxis left
plotReducedSignal(t,Ez,N,'-k',4,'Time (sec)','Error (E_z in m & E_\theta in rad)','E_z',"Error_Plot");
% yyaxis right
plotReducedSignal(t,Epitch,N,'-.k',3,'Time (sec)','Error (E_z in m & E_\theta in rad)','E_\theta',"Error_Plot");


exportFigure('Error_Plot');
pause(3)
close()
%% Depth tracking
figure('Units','normalized','OuterPosition',[0 0 1 1])
plotReducedSignal(t,Zm,N,'-k',4,'Time (sec)','Depth (m)','Z',"Depth_Tracking");
% plotReducedSignal(t,Etheta,N,'-.k',3,'Time (sec)','Error','E_\theta',"Results/fig/2026/March/13032026/new1.fig");

exportFigure('Depth_tacking');
pause(3)
close()


%% Pitchtracking
filename = "Pitch_Tracking";
figure('Units','normalized','OuterPosition',[0 0 1 1])
plotReducedSignal(t,pitch,N,'-k',4,'Time (sec)','Pitch (\theta rad)','\theta',filename);

exportFigure(filename);
pause(3)
close()

%% Pitchtracking degree
filename = "Pitch_Tracking_degree";
figure('Units','normalized','OuterPosition',[0 0 1 1])
plotReducedSignal(t,pitchd,N,'-k',4,'Time (sec)','Pitch (\theta rad)','\theta',filename);

exportFigure(filename);
pause(3)
close()


%% Force tracking
filename='Force';
figure('Units','normalized','OuterPosition',[0 0 1 1])
plotReducedSignal(t,u1,N,'-k',4,'Time (sec)','Force (N)','Front Left',filename);
plotReducedSignal(t,u1,N,'--k',4,'Time (sec)','Force (N)','Front Right',filename);
plotReducedSignal(t,u2,N,'-.k',4,'Time (sec)','Force (N)','Rear',filename);


exportFigure(filename)

pause(3)
close()

%% Syringe tracking
filename = 'Syringe_position';
figure('Units','normalized','OuterPosition',[0 0 1 1])
plotReducedSignal(t,s1l,N,'-k',4,'Time (sec)','Pose (m)','Front Left',filename);
plotReducedSignal(t,s1r,N,'--b',4,'Time (sec)','Pose (m)','Front Right',filename);
plotReducedSignal(t,s2,N,'-.k',4,'Time (sec)','Pose (m)','Rear',filename);

exportFigure(filename);
pause(3)
close()