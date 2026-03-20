function exportFigure(filename)

global FIG_SAVE_PATH

% ensure folder exists
if ~exist(FIG_SAVE_PATH,'dir')
    mkdir(FIG_SAVE_PATH)
end

% remove extension if user provided one
[~,name,~] = fileparts(filename);

% construct paths
epsFile = fullfile(FIG_SAVE_PATH,[name,'.eps']);
pdfFile = fullfile(FIG_SAVE_PATH,[name,'.pdf']);

% export
exportgraphics(gcf,epsFile,'ContentType','vector','BackgroundColor','white')
exportgraphics(gcf,pdfFile,'ContentType','vector')

end