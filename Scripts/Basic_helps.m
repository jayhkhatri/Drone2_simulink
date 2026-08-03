%% root file for basic things like video recording and workspace saving at proper location.

% HybridDroneProject/
% ├── Models/
% │   └── hybrid_model.slx
% ├── Scripts/
% │   └── control_scripts.m
% ├── Functions/
% │   └── helperFunctions.m
% ├── Geometry/              ← STL files go here
% │   ├── frame.stl
% │   ├── propeller.stl
% │   └── motor_mount.stl
% ├── Data/
% │   └── parameters.mat
% ├── Docs/
% │   └── README.md
% └── HybridDroneProject.prj


%% saving of .mat file to Results/Mat
% Get the root folder of your current MATLAB Project
proj = matlab.project.rootProject;
rootDir = proj.RootFolder;

% Define the path to your target folder
saveDir = fullfile(rootDir, 'results', '2026','Jan','MAT');  %rootDir/results/MAT

% Make sure the folder exists (create it if not)
if ~exist(saveDir, 'dir')
    mkdir(saveDir);
end

% Now save your data
save(fullfile(saveDir, 'ResultsM2_5toM5.mat'),"out");

%% saving of image file in results/images

% Example variable to plot
x = 0:0.1:10;
y = sin(x);

% Get project root
proj = matlab.project.rootProject;
rootDir = proj.RootFolder;

% Define image save path
imgDir = fullfile(rootDir, 'results', 'images');   %%rootDir/results/images
if ~exist(imgDir, 'dir')
    mkdir(imgDir);
end

% Create and save plot
fig = figure;
plot(x, y);
title('Sine Wave');

% Save as .tiff
saveas(fig, fullfile(imgDir, 'sine_wave.tiff'));


%% saving videos from mechanical exploler in videos
% Define output folder inside the MATLAB Project
proj = matlab.project.rootProject;
rootDir = proj.RootFolder;
videoDir = fullfile(rootDir, 'videos');

% Ensure the folder exists
if ~exist(videoDir, 'dir')
    mkdir(videoDir);
end

% Define video output file
videoFile = fullfile(videoDir, 'mechanics_animation.avi');

% Set simulation parameters for video recording
set_param('YourModelName', 'SimMechanicsOpenEditorOnUpdate', 'on');
set_param('YourModelName', 'SimMechanicsOpenEditorOnUpdate', 'on');

% Open Mechanics Explorer with video recording enabled
sim('YourModelName'); % Run simulation

% After simulation, export animation manually OR:
smwritevideo('YourModelName', videoFile, 'PlaybackSpeedRatio', 1);


%% to save script (.m) either use this code or just copy pase the file in project.
proj = matlab.project.rootProject;
rootDir = proj.RootFolder;

targetDir = fullfile(rootDir, 'scripts');
if ~exist(targetDir, 'dir')
    mkdir(targetDir);
end

filePath = fullfile(targetDir, 'my_new_script.m');
fid = fopen(filePath, 'w');
fprintf(fid, 'disp("This is a new script inside the project.")\n');
fclose(fid);


%% to save script (.slx) either use this code or just copy pase the file in project.
proj = matlab.project.rootProject;
rootDir = proj.RootFolder;

modelDir = fullfile(rootDir, 'models');
if ~exist(modelDir, 'dir')
    mkdir(modelDir);
end

modelName = 'new_model';
newModelPath = fullfile(modelDir, [modelName, '.slx']);

new_system(modelName); % create new model
open_system(modelName);
save_system(modelName, newModelPath); % save to desired location
close_system(modelName, 0); % close without prompt
%% clubing videos

% Define paths to the 4 videos
v1 = VideoReader('Hover_Frontview.avi');
v2 = VideoReader('Hover_Sideview.avi');
v3 = VideoReader('Hover_Topview.avi');
v4 = VideoReader('Hover_Isoview.avi');

% Set up the output video
outputVideo = VideoWriter('Hover_Cobined.avi');
outputVideo.FrameRate = min([v1.FrameRate, v2.FrameRate, v3.FrameRate, v4.FrameRate]);
open(outputVideo);

% Define minimum number of frames (to sync all)
numFrames = min([v1.NumFrames, v2.NumFrames, v3.NumFrames, v4.NumFrames]);

% Loop through each frame
for k = 1:numFrames
    % Read frame from each video
    f1 = read(v1, k);
    f2 = read(v2, k);
    f3 = read(v3, k);
    f4 = read(v4, k);
    
    % Resize all frames to same size
    targetSize = [min([size(f1,1), size(f2,1), size(f3,1), size(f4,1)]), ...
                  min([size(f1,2), size(f2,2), size(f3,2), size(f4,2)])];
    
    f1 = imresize(f1, targetSize);
    f2 = imresize(f2, targetSize);
    f3 = imresize(f3, targetSize);
    f4 = imresize(f4, targetSize);
    
    % Create 2x2 grid
    topRow = [f1, f2];
    bottomRow = [f3, f4];
    combinedFrame = [topRow; bottomRow];
    
    % Write frame to output
    writeVideo(outputVideo, combinedFrame);
end

close(outputVideo);
disp('✅ Combined video saved as combined_2x2.avi');
%% collision viewer
TR = stlread('Drone_collisionmodel.STL');
% extract verices
P = TR.Points;
% comput convex hull
K = convhull(P(:,1),P(:,2),P(:,3));
%plot orginal mess
figure

trisurf(TR.ConnectivityList,...
        P(:,1),P(:,2),P(:,3),...
        'FaceColor',[0.7 0.7 0.7],...
        'EdgeColor','none');

axis equal
camlight
lighting gouraud
title('Original Geometry')

% plot convex hull
figure

trisurf(K,...
        P(:,1),P(:,2),P(:,3),...
        'FaceColor','red',...
        'FaceAlpha',0.4,...
        'EdgeColor','black');

axis equal
camlight
lighting gouraud
title('Convex Hull')
% both
figure
hold on

% Original
trisurf(TR.ConnectivityList,...
        P(:,1),P(:,2),P(:,3),...
        'FaceColor',[0.7 0.7 0.7],...
        'FaceAlpha',0.25,...
        'EdgeColor','none');

% Convex Hull
trisurf(K,...
        P(:,1),P(:,2),P(:,3),...
        'FaceColor','red',...
        'FaceAlpha',0.5,...
        'EdgeColor','black');

axis equal
camlight
lighting gouraud
rotate3d on