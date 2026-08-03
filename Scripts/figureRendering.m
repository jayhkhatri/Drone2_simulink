global basepath FIG_SAVE_PATH;

proj = currentProject;
path = proj.RootFolder;
basepath = fullfile(proj.RootFolder,'Results','fig','2026','May','06052026');
FIG_SAVE_PATH = fullfile(proj.RootFolder,'Results','epspdf','2026','May','06052026');
MATfilePath = fullfile(proj.RootFolder,"Results","MAT");

Newfig = 1;
%% Savind Data in .MAT file

if ~exist(MATfilePath, 'dir')
    mkdir(MATfilePath);
end

filename = sprintf('Results1_with_SimpleBackstepping_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')); %'Results_with_New_STA_Controller_%s.mat'

save(fullfile(MATfilePath, filename), 'out');

%% to load file   %% use only when need to run data from .MAT file rather than simuluink
% file_name = "Results1_with_SimpleBackstepping_20260502_144103.mat";
% file_name = "Results_with_New_STA_Controller_20260502_131828.mat";
%'Results_with_SimpleBackstepping_20260502_115955.mat'
file_name = "Results1_with_SimpleBackstepping_20260504_105839.mat";
file = fullfile(MATfilePath,file_name);
AA = load(file);


out = AA.out;  % use this when need to draw graph from mat file and not
%from the simulink out put.

%% 
t = out.tout;
Yindex = ones(length(t),1);
Xm = out.data.COM.World.Xe.X.Data;
Ym = out.data.COM.World.Xe.Y.Data;
Zm  = out.data.COM.World.Xe.Z.Data;
roll = out.data.COM.Derived.Euler.phi.Data;
pitch = out.data.COM.Derived.Euler.theta.Data;
yaw = out.data.COM.Derived.Euler.psi.Data;
zdesired = out.zdesired.Data;
%% steady state data
zm1 = Zm(300000:600000);
zm2 = Zm(900000:1200000);
ZC = [zm1;zm2];

pm1 = pitch(300000:60000);
pm2 = pitch(900000:1200000);
PC = [pm1;pm2];
%% in degree% %
rolld = rad2deg(roll);
pitchd = rad2deg(pitch);
yawd = rad2deg(yaw);
%% to take data from error
edata = squeeze(out.error.Data);  % CONVERTS 3D ARRAY TO 2D BY REMOVING TIME
Ez = edata(1,:)';
Epitch = edata(2,:)';
%% ld and bd data
lddata = squeeze(out.ld.Data);
bdata = squeeze(out.bout.Data);
ld1 = lddata(1,:);
ld2 = lddata(2,:);
b1 = bdata(1,:);
b2 = bdata(2,:);

%% to take data from force
fdata = squeeze(out.force.Data);  % CONVERTS 3D ARRAY TO 2D BY REMOVING TIME
u1 = fdata(1,:)';
u2 = fdata(2,:)';

%% cost parameters
IAEdata = squeeze(out.Cost.IAE.Data);
ISUdata = squeeze(out.Cost.ISU.Data);
RMSdata = squeeze(out.Cost.RMS.Data);
TVdata = squeeze(out.Cost.Smoothness.Data);
%% display
format long g
RMS = RMSdata(end)
ISU = ISUdata(end)
IAE = IAEdata(end)
totalVariation = TVdata(end)
%% performace data
MSE_z = out.MSE_ez.Data(end)
MSE_theta = out.MSE_etheta.Data(end)

IAEZ = out.IAE_ez.Data(end)
IAEtheta = out.IAE_etheta.Data(end)

ISEz = out.ISE_ez.Data(end)
ISEtheta = out.ISE_etheta.Data(end)

ITAEz = out.ITAE_ez.Data(end)
ITAEtheta = out.ITAE_etheta.Data(end)
%% syringe data

s1l = out.data.Buoyancy.Syringe.SyringeFL.Pose.Data;
s1r = out.data.Buoyancy.Syringe.SyringeFR.Pose.Data;
s2 = out.data.Buoyancy.Syringe.SyringeB.Pose.Data;


%% plotiing function basic data
N = 1201; % sample points
type = 'With BS}';

%% plotting error plot
filename = "Error_Plot_BS";
figure('Units','normalized','OuterPosition',[0 0 1 1])

% rectangle('Position',[0 -0.15 1200 0.3],'FaceColor',"g",FaceAlpha=0.1,EdgeColor=[1 1 1]);
plotReducedSignal(t,Epitch,N,'-.k',3,'Time (sec)','Error (E_z in m & E_\theta in rad)','E_\theta',filename);
% yyaxis left
plotReducedSignal(t,Ez,N,'-k',3,'Time (sec)','Error (E_z in m & E_\theta in rad)','E_z',filename);
% yyaxis right

%%


exportFigure(filename);
pause(3)
close()

%% Depth tracking
filename = "Depth_Tracking";
if Newfig
figure('Units','normalized','OuterPosition',[0 0 1 1])
plotReducedSignal(t,zdesired,N,'-b',4,'Time (sec)','Depth (m)','Z_{desired}',filename); %desired
plotReducedSignal(t,Zm,N,'-k',4,'Time (sec)','Depth (m)',sprintf('%s %s','Z_{measured',type),filename); % measured
pause(3)
close()

else
fig = openfig(fullfile(basepath,[char(filename),'.fig']),"reuse");
plotReducedSignal(t,Zm,N,'-m',4,'Time (sec)','Depth (m)',sprintf('%s %s','Z_{measured',type),"Depth_Tracking"); % measured
end
%%
exportFigure(filename);
close()


%% Pitchtracking
filename = "Pitch_Tracking";

if Newfig
figure('Units','normalized','OuterPosition',[0 0 1 1])
plotReducedSignal(t,zeros(length(t),1),N,'-.k',2,'Time (sec)','Pitch (\theta rad)','\theta_{desired}',filename);  % desired
plotReducedSignal(t,pitch,N,'-k',3,'Time (sec)','Pitch (\theta rad)',sprintf('%s %s','\theta_{Measured',type),filename);  % measured
pause(3)
close()

else  
fig = openfig(fullfile(basepath,[char(filename),'.fig']),"reuse");
plotReducedSignal(t,pitch,N,'--k',3,'Time (sec)','Pitch (\theta rad)',sprintf('%s %s','\theta_{Measured',type),filename);  % measured
end
%%
exportFigure(filename);
close()


%% Pitchtracking degree
filename = "Pitch_Tracking_degree";
if Newfig
figure('Units','normalized','OuterPosition',[0 0 1 1])
plotReducedSignal(t,zeros(length(t),1),N,'-b',3,'Time (sec)','Pitch (\theta^\circ)','\theta_{desired}',filename); % desired
plotReducedSignal(t,pitchd,N,'-k',3,'Time (sec)','Pitch(\theta^\circ)',sprintf('%s %s','\theta_{Measured',type),filename); % Measured
pause(5)
close()


else
fig = openfig(fullfile(basepath,[char(filename),'.fig']),"reuse");
plotReducedSignal(t,pitchd,N,'--k',3,'Time (sec)','Pitch (\theta^\circ)',sprintf('%s %s','\theta_{Measured',type),filename);  % measured  
end
%%
exportFigure(filename);

close()

%% Force tracking
filename='Force_STA';

if Newfig
    figure('Units','normalized','OuterPosition',[0 0 1 1])
    view(3)
    plotReducedSignal3d(t,Yindex,u1,N,'-',[0 0 0],3,'Time (sec)','Index','Force (N)',sprintf('%s %s','Front Left_{',type),filename);
    plotReducedSignal3d(t,2*Yindex,u1,N,'-',[0 0 1],3,'Time (sec)','Index','Force (N)',sprintf('%s %s','Front Right_{',type),filename);
    plotReducedSignal3d(t,3*Yindex,u2,N,'-.',[1 0 0],3,'Time (sec)','Index','Force (N)',sprintf('%s %s','Rear_{',type),filename);
    % pause(3)
    % close()
else
    % fig = openfig(fullfile(basepath,[char(filename),'.fig']),"reuse"); 
    figure('Units','normalized','OuterPosition',[0 0 1 1])
    view(3)
    plotReducedSignal3d(t,4*Yindex,u1,N,'-',[0.9290 0.6940 0.1250],3,'Time (sec)','Index','Force (N)',sprintf('%s %s','Front Left_{',type),filename);
    plotReducedSignal3d(t,5*Yindex,u1,N,'-',[0.4940 0.1840 0.5560],3,'Time (sec)','Index','Force (N)',sprintf('%s %s','Front Right_{',type),filename);
    plotReducedSignal3d(t,6*Yindex,u2,N,':',[1 0 0],3,'Time (sec)','Index','Force (N)',sprintf('%s %s','Rear_{',type),filename);
end

%%
exportFigure(filename)
close()
%% ld and b tracting

filename = 'ld_tracking_BS';
if Newfig
    figure('Units','normalized','OuterPosition',[0 0 1 1])
    view(3)
    plotReducedSignal3d(t,Yindex,ld1,N,'-',[0 0 0],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','ld Front Left_{',type),filename);
    plotReducedSignal3d(t,2*Yindex,ld1,N,'-',[0 0 1],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','ld Front Right_{',type),filename);
    plotReducedSignal3d(t,3*Yindex,ld2,N,'-.',[1 0 0],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','ld Rear_{',type),filename);
    % pause(3)
    % close()
else
    % fig = openfig(fullfile(basepath,[char(filename),'.fig']),"reuse"); 
    figure('Units','normalized','OuterPosition',[0 0 1 1])
    view(3)
    plotReducedSignal3d(t,4*Yindex,ld1,N,'-',[0.9290 0.6940 0.1250],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','ld Front Left_{',type),filename);
    plotReducedSignal3d(t,5*Yindex,ld1,N,'-',[0.4940 0.1840 0.5560],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','ld Front Right_{',type),filename);
    plotReducedSignal3d(t,6*Yindex,ld2,N,':',[1 0 0],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','ld Rear_{',type),filename);
end

%%
filename = 'b_Tracking_BS';
if Newfig
    figure('Units','normalized','OuterPosition',[0 0 1 1])
    view(3)
    plotReducedSignal3d(t,Yindex,b1,N,'-',[0 0 0],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','b Front Left_{',type),filename);
    plotReducedSignal3d(t,2*Yindex,b1,N,'-',[0 0 1],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','b Front Right_{',type),filename);
    plotReducedSignal3d(t,3*Yindex,b2,N,'-.',[1 0 0],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','b Rear_{',type),filename);
    % pause(3)
    % close()
else
    % fig = openfig(fullfile(basepath,[char(filename),'.fig']),"reuse"); 
    figure('Units','normalized','OuterPosition',[0 0 1 1])
    view(3)
    plotReducedSignal3d(t,4*Yindex,b1,N,'-',[0.9290 0.6940 0.1250],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','b Front Left_{',type),filename);
    plotReducedSignal3d(t,5*Yindex,b1,N,'-',[0.4940 0.1840 0.5560],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','b Front Right_{',type),filename);
    plotReducedSignal3d(t,6*Yindex,b2,N,':',[1 0 0],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','b Rear_{',type),filename);
end

%% Syringe tracking
filename = 'Syringe_position_BS';
if Newfig
    figure('Units','normalized','OuterPosition',[0 0 1 1])
    view(3)
    plotReducedSignal3d(t,Yindex,s1l,N,'-',[0 0 0],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','Front Left_{',type),filename);
    plotReducedSignal3d(t,2*Yindex,s1r,N,'-',[0 0 1],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','Front Right_{',type),filename);
    plotReducedSignal3d(t,3*Yindex,s2,N,'-.',[1 0 0],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','Rear_{',type),filename);
    % pause(3)
    % close()
else
    % fig = openfig(fullfile(basepath,[char(filename),'.fig']),"reuse"); 
    figure('Units','normalized','OuterPosition',[0 0 1 1])
    view(3)
    plotReducedSignal3d(t,4*Yindex,s1l,N,'-',[0.9290 0.6940 0.1250],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','Front Left_{',type),filename);
    plotReducedSignal3d(t,5*Yindex,s1r,N,'-',[0.4940 0.1840 0.5560],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','Front Right_{',type),filename);
    plotReducedSignal3d(t,6*Yindex,s2,N,':',[1 0 0],3,'Time (sec)','Index','Position (m)',sprintf('%s %s','Rear_{',type),filename);
end
%%
exportFigure(filename);
close()

%% fft and high frequcy energy ratio
u = [u1 u2];
thresold = 0.04;
filename = 'fftresponse';
% operate any one based on requirement
% [HF_ratio_STAf1, OA_STAf1, zcr_STAf1, DE_STAf1] = fftresponse(u1,1000,thresold,'-k',3,'FFT STAf1',filename,Newfig);
% [HF_ratio_STAf2, OA_STAf2, zcr_STAf2, DE_STAf2] = fftresponse(u1,1000,thresold,'--b',3,'FFT STAf2',filename,false);
% [HF_ratio_STAr, OA_STAr, zcr_STAr, DE_STAr] = fftresponse(u2,1000,thresold,'-r',3,'FFT STAr',filename,false);
[HF_ratio_BSf1, OA_BSf1, zcr_BSf1, DE_BSf1] = fftresponse(u1,1000,thresold,'--k',3,'FFT BSf1',filename,Newfig)
[HF_ratio_BSf2, OA_BSf2, zcr_BSf2, DE_BSf2] = fftresponse(u1,1000,thresold,'--g',3,'FFT BSf2',filename,Newfig)
[HF_ratio_BSr, OA_BSr, zcr_BSr, DE_BSr] = fftresponse(u2,1000,thresold,':r',3,'FFT BSr',filename,Newfig)


% [HF_ratio_BS1, OA_BS1, zcr_BS1, DE_BS1] = fftresponse(u1,1000,0.01,'--m',3,'FFT BS',filename,Newfig);

%% 
exportFigure(filename);

%% for taking zoom option
ax = gca;

disp('Click two opposite corners of zoom region')
[x, y] = ginput(2);
%%
% Extract limits
x1 = min(x); x2 = max(x);
y1 = min(y); y2 = max(y);

% Draw rectangle on main plot
rectangle('Position',[x1 y1 (x2-x1) (y2-y1)], ...
          'EdgeColor','k','LineStyle','--')

% Create inset
ax2 = axes('Position',[0.45 0.40 0.3 0.3]);
box on; hold on;grid on; grid minor;

% Copy all lines automatically
lines = findobj(ax,'Type','line');
for i = 1:length(lines)
    plot(lines(i).XData, lines(i).YData, ...
        'Color', lines(i).Color, ...
        'LineStyle', lines(i).LineStyle, ...
        'LineWidth', lines(i).LineWidth);
end

xlim([x1 x2])
ylim([y1 y2])

set(ax2,'FontSize',20)

%%

ZRMSE_BS = compute_rmse(Zm,zdesired)
PRMSE_BS = compute_rmse(pitch,zeros(length(pitch),1))
ZSmoothness_STA2 = smoothness_second_diff(ZC)
ZSmoothness_STA1 = smoothness_diff(ZC)
PSmoothness_STA2 = smoothness_second_diff(PC)
PSmoothness_STA1 = smoothness_diff(PC)
%%
filename = 'XZ_plot';

% figure('Units','normalized','OuterPosition',[0 0 1 1])
fig = openfig(fullfile(basepath,[char(filename),'.fig']),"reuse");
%%
plotReducedSignal(Xm,Zm,1000,'-r',2,'X (m)','Z (m)','Tracking_{BS}',filename);
%%
exportFigure(filename);
close

%% 
img =im2double(imread("images\Picture4.png"));
a = min(img(:));b =  max(img(:));
%%
scale = 0.25;
step = 50000;
for k=1:step:length(t)
    xi = Xm(k);
    zi = Zm(k);

    image([xi-scale xi+scale], [zi-scale zi+scale],img)
end
%%
[img,~,alpha] = imread("images\Picture3.png");
%% 
alpha1 = any(img <0.95, 3);

%%
imshow(img);
%%
hold on
h=imshow(img);
set(h,'AlphaData',alpha);