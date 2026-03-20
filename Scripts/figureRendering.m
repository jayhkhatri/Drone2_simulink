global basepath FIG_SAVE_PATH;

proj = currentProject;
path = proj.RootFolder;
basepath = fullfile(proj.RootFolder,'Results','fig','2026','March','13032026','lowvelocity');
FIG_SAVE_PATH = fullfile(proj.RootFolder,'Results','epspdf','2026','March','13032026','lowvelocity');




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

s1l = out.data.Buoyancy.Syringe.SyringeFL.Pose;
s1r = out.data.Buoyancy.Syringe.SyringeFR.Pose;
s2 = out.data.Buoyancy.Syringe.SyringeB.Pose;


%% plotiing function

%% plotting error plot
N = 3001; % sample points
figure(1)
rectangle('Position',[0 -0.15 1200 0.3],'FaceColor',"g",FaceAlpha=0.1,EdgeColor=[1 1 1]);
% yyaxis left
plotReducedSignal(t,Ez,N,'-k',4,'Time (sec)','Error (E_z in m & E_\theta in rad)','E_z',"Results/fig/2026/March/13032026/new.fig");
% yyaxis right
plotReducedSignal(t,Epitch,N,'-.k',3,'Time (sec)','Error (E_z in m & E_\theta in rad)','E_\theta',"Results/fig/2026/March/13032026/new.fig");
close(1)
%%
figure(2)
plotReducedSignal(t,Zm,N,'-k',4,'Time (sec)','Depth (m)','E_z',"Results/fig/2026/March/13032026/new1.fig");
% plotReducedSignal(t,Etheta,N,'-.k',3,'Time (sec)','Error','E_\theta',"Results/fig/2026/March/13032026/new1.fig");
close(2)
