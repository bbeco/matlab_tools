clear VARIABLES;
% setting current folder
cd(fullfile('../'));
addpath(fullfile('coordinate_transform'));
addpath(fullfile('utils'));
addpath(fullfile('filters'));
addpath(fullfile('ground_truth'));
addpath(fullfile('data_analysis'));
baseDir = fullfile('images/sfm_test/test23_piazzaLeoni_empoli1/');
imageDir = fullfile(baseDir, '*.jpg');

% result base directory
resultBaseFolder = fullfile('results/realSeqSfmTest/test23/');
if exist(resultBaseFolder, 'dir') == 0
	mkdir(resultBaseFolder);
end

imds = imageDatastore(imageDir);
% start with image...
firstFrame = 400;
% and stop with image...
lastFrame = 800;

% ********** PARAMETERS ************
% whether to plot camera position or not
enableFigures = true;
repetitions = 1;

computeRelativeScaleBeforeBundleAdjustment = true;
maxAcceptedReprojectionError = 0.8;

% filter those matches whose points have similar coordinates
filterMatches = true;
angularThreshold = 2; %degrees

% minimun threshold for the Z coordinate of the key points directions.
zMin = 0.04;

% remove keypoints that are too close to the poles
prefilterLLKeyPoints = true;
maxLatitudeAngle = 60; %degrees

% Bundle Adjustment parameters
performGlobalBundleAdjustment = true;
bundleAdjustmentAbsoluteTolerance = 1e-05;
bundleAdjustmentRelativeTolerance = 1e-09;
bundleAdjustmentMaxIterations = 300;

% Windowed bundle adjustment parameters
performWindowedBundleAdjustment = true;
% see paramTable for this
windowSize = 5;
windowAngularThreshold = 5; %degrees

% Sequence filter parameters
% ... see experiment params
seqFilterQuantile = 0.8;

% setting seed
rng('default');

% **********************************

%% Camera trajectory and sparse point cloud estimating function

statusFileName = fullfile(resultBaseFolder, 'workspace.mat');
if exist(statusFileName, 'file')
	status = load(statusFileName);
	vSet = status.vSet;
	xyzPoints = status.xyzPoints;
	frameUsed = status.frameUsed; 
else
	[vSet, xyzPoints, ~, ~, ~, ~, frameUsed] = ...
			sfmLL_function(imageDir, ...
			computeRelativeScaleBeforeBundleAdjustment, ...
			maxAcceptedReprojectionError, filterMatches, angularThreshold, ...
			zMin, ...
			prefilterLLKeyPoints, maxLatitudeAngle, ...
			performGlobalBundleAdjustment, performWindowedBundleAdjustment, ...
			windowSize, firstFrame, lastFrame, ...
			bundleAdjustmentAbsoluteTolerance, ...
			bundleAdjustmentRelativeTolerance, bundleAdjustmentMaxIterations,...
			windowAngularThreshold, seqFilterQuantile);
	% Saving environment
	save(fullfile(resultBaseFolder, 'workspace.mat'));
end

% *************************************

%% Densification: dense point cloud reconstruction
% extracting camera's poses
% poses = vSet.poses();
% poses = translateLocation(poses);
% poses = alignOrientation(poses);

reconstructing_function(xyzPoints, vSet.poses(), frameUsed, baseDir, resultBaseFolder);
%**************************************

%% Saving environment
if exist(statusFileName, 'file') ~= 0
	delete(statusFileName);
end
save(statusFileName);

disp(['You can find the results in ', resultBaseFolder ]);
