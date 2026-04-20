function plotReducedSignal(x,y,N,linetype,linewidth,xlab,ylab,legtext,filename)

global basepath
filename = char(filename);

if ~exist(basepath,'dir')
    mkdir(basepath)
end


[~,name,~] = fileparts(filename);
% proj = currentProject;
% rootPath = proj.RootFolder;

idx = round(linspace(1,length(x),N));

% Create figure only if none exists
if isempty(get(groot,'CurrentFigure'))
    fig = figure;
else
    fig = gcf;
end

hold on
set(gcf,'Renderer','painters')
plot(x(idx),y(idx),linetype,'LineWidth',linewidth,'DisplayName',legtext)


%% axis formating
ax = gca;

ax.FontSize = 40;          % axis font size
ax.FontName = 'Times New Roman';

ax.XMinorGrid = 'off';      % minor grid
ax.YMinorGrid = 'on';

ax.GridAlpha = 0.4;
ax.MinorGridAlpha = 0.25;

%% legend formating

lgd = legend;
lgd.Interpreter = "tex";
lgd.FontSize = 36;
lgd.FontName = 'Times New Roman';
lgd.Location ='best';




grid on

xlabel(xlab,'FontSize',44)
ylabel(ylab,'FontSize',44)

set(gca,'SortMethod','childorder')
set(gcf,'Renderer','painters')


% if ~isempty(filename)
%     [filepath,name,ext] = fileparts(filename);
% 
%     if isempty(ext)
%         ext = '.fig';
%     end
% 
%     if isempty(filepath)
%         if isempty(basepath)
%             warning("No basepath provided. file will not be saved.");
%             return;
%         end
% 
%     else
%         savepath = fullfile(filepath,[name ext]);
%     end
% 
%     folder = fileparts(savepath);
%     if isempty(folder)
%         warning("folder path empty. FIll will not be saved");
%         return
%     end
%     if ~exist(folder,'dir')
%         mkdir(folder)
%     end
% 
%     savefig(fig,savepath,"compact");
% end


savePath = fullfile(basepath,[name,'.fig']);

folder = fileparts(savePath);
    if isempty(folder)
        warning("folder path empty. FIll will not be saved");
        return
    end

if ~exist(folder,'dir')
    mkdir(folder)
end

savefig(fig,savePath,'compact')

end