clear VARIABLES;
addpath('coordinate_transform');
addpath('utils/');
addpath('filters/');
addpath('ground_truth');
imageDir = fullfile('images', 'sfm_test', 'test4', {'ll0.png', 'll1.png'});
load(fullfile('images', 'sfm_test', 'test4', 'groundTruth.mat'));
filename = '../test4.xlsx';

% ********** PARAMETERS ************
% whether to plot camera position or not
enableFigures = true;
repetitions = 1;

computeRelativeScaleBeforeBundleAdjustment = true;
maxAcceptedReprojectionError = 0.8;

% filter those matches whose points have similar coordinates
filterMatches = false;
angularThreshold = 2; %degrees

% minimun threshold for the third components of key points directions.
% This is used when the keypoints are not projected on a new image plane for 
% essential matrix estimation
zMin = 0.037;

prefilterLLKeyPoints = false;
maxLatitudeAngle = 60; %degrees

performBundleAdjustment = false;

% This is the number of views for a keypoint to appear into in order for it to
% be added in a connection's match.
viewsWindowSize = 2;
% **********************************

imds = imageDatastore(imageDir);
c = numel(imds.Files);
relLocationError = cell(repetitions, 1);
relOrientationError = cell(repetitions, 1);
pointsForEEstimationCounter = cell(repetitions, 1);
pointsForPoseEstimationCounter = cell(repetitions, 1);
locError = cell(repetitions, c);
locErrorNorm = zeros(repetitions, c);
orientError = cell(repetitions, c);
locXerror = zeros(repetitions, c);
locYerror = zeros(repetitions, c);
locZerror = zeros(repetitions, c);
angularXerror = zeros(repetitions, c);
angularYerror = zeros(repetitions, c);
angularZerror = zeros(repetitions, c);

distanceGT = zeros(c, 1);
orientationGT = zeros(c, 3);

groundTruthPoses = alignOrientation(groundTruthPoses);


for i = 1:repetitions

	[vSet, xyzPoints, reprojectionErrors, relLocation, relOrientation, ...
		pointsForEEstimationCounter{i}, ...
		pointsForPoseEstimationCounter{i}] = ...
		sfmLL_function(imageDir, ...
		computeRelativeScaleBeforeBundleAdjustment, ...
		maxAcceptedReprojectionError, filterMatches, angularThreshold, ...
		zMin, ...
		prefilterLLKeyPoints, maxLatitudeAngle, ...
		performBundleAdjustment, viewsWindowSize, groundTruthPoses);

	[vSet, groundTruthPoses] = normalizeViewSet(vSet, groundTruthPoses);
	camPoses = poses(vSet);
	
	estLocation = camPoses.Location;
	estOrientation = camPoses.Orientation;
	[tmpLocError, tmpOrientError, tmpRelLocError, tmpRelOrientError] = ...
		computePoseError(estLocation, estOrientation, groundTruthPoses, [1 2]);
		
	for j = 1:size(camPoses, 1)
		locError{i, j} = tmpLocError{j};
		locErrorNorm(i, j) = norm(locError{i, j});

		orientError{i, j} = 180/pi*tmpOrientError{j};
		angularZerror(i, j) = orientError{i, j}(1);
		angularYerror(i, j) = orientError{i, j}(2);
		angularXerror(i, j) = orientError{i, j}(3);
	end
	
	tmp = cat(1, locError{i, :});
	locXerror(i, :) = tmp(:, 1)';
	locYerror(i, :) = tmp(:, 2)';
	locZerror(i, :) = tmp(:, 3)';
end

orientationGT = 180/pi*orientationGT;

%% Write results
params = table(repetitions, computeRelativeScaleBeforeBundleAdjustment, ...
	maxAcceptedReprojectionError, filterMatches, angularThreshold, ...
	zMin,...
	prefilterLLKeyPoints, maxLatitudeAngle, performBundleAdjustment, ...
	viewsWindowSize);

writetable(params, filename, 'Range', 'A1');

tmp = cat(1, groundTruthPoses.Location{:});

%groundTruth locX
columnNames = ...
	{'locGTX'};
groundTruthTable = table(tmp(:, 1)', ...
	'VariableNames', columnNames);
writetable(groundTruthTable, filename, 'Range', 'A5');

base = 5;
tSize = 4;
tableNumber = 1;

%groundTruth locY
columnNames = {'locGTY'};
errorLocationTable = table(tmp(:, 2)', 'VariableNames', ...
	columnNames);
writetable(errorLocationTable, filename, 'Range', ...
	['A', num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

%groundTruth locZ
columnNames = {'locGTZ'};
errorLocationTable = table(tmp(:, 3)', 'VariableNames', ...
	columnNames);
writetable(errorLocationTable, filename, 'Range', ...
	['A', num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

orientationsGT = orientations2euler(groundTruthPoses.Orientation);
tmp = cat(1, orientationsGT{:});

%groundTruth orientX
columnNames = {'orientGTX'};
errorLocationTable = table(tmp(:, 3)', 'VariableNames', ...
	columnNames);
writetable(errorLocationTable, filename, 'Range', ...
	['A', num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

%groundTruth orientY
columnNames = {'orientGTY'};
errorLocationTable = table(tmp(:, 2)', 'VariableNames', ...
	columnNames);
writetable(errorLocationTable, filename, 'Range', ...
	['A', num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

%groundTruth orientZ
columnNames = {'orientGTZ'};
errorLocationTable = table(tmp(:, 1)', 'VariableNames', ...
	columnNames);
writetable(errorLocationTable, filename, 'Range', ...
	['A', num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

base = base + tableNumber*tSize;
tSize = repetitions + 3;
tableNumber = 0;
%location error
columnNames = {'repetition', 'locError'};
errorLocationTable = table((1:repetitions)', locErrorNorm, 'VariableNames', ...
	columnNames);
writetable(errorLocationTable, filename, 'Range', ...
	['A', num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

%angular error X
errorXtable = table((1:repetitions)', angularXerror, ...
	'VariableNames', {'repetitions', 'orientX_deg'});
writetable(errorXtable, filename, 'Range', ...
	['A' num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

%angular error Y
errorYtable = table((1:repetitions)', angularYerror, ...
	'VariableNames', {'repetitions', 'orientY_deg'});
writetable(errorYtable, filename, 'Range', ...
	['A' num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

%angular error Z
errorZtable = table((1:repetitions)', angularZerror, ...
	'VariableNames', {'repetitions', 'orientZ_deg'});
writetable(errorZtable, filename, 'Range', ...
	['A' num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

% Estimation data
writetable(...
	table(...
	(1:repetitions)', pointsForEEstimationCounter, ...
	'VariableNames', {'repetitions', 'pointsForEEstimation'}), ...
	filename, 'Range', ['A' num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

writetable(...
	table(...
	(1:repetitions)', pointsForPoseEstimationCounter, ...
	'VariableNames', {'repetitions', 'pointsForPoseEstimation'}), ...
	filename, 'Range', ['A' num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

%location error X
columnNames = {'repetition', 'locXerror'};
errorLocationTable = table((1:repetitions)', locXerror, ...
	'VariableNames', columnNames);
writetable(errorLocationTable, filename, 'Range', ...
	['A' num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

%location error Y
columnNames = {'repetition', 'locYerror'};
errorLocationTable = table((1:repetitions)', locYerror, ...
	'VariableNames', columnNames);
writetable(errorLocationTable, filename, 'Range', ...
	['A' num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

%location error Z
columnNames = {'repetition', 'locZerror'};
errorLocationTable = table((1:repetitions)', locZerror, ...
	'VariableNames', columnNames);
writetable(errorLocationTable, filename, 'Range', ...
	['A' num2str(base + tableNumber*tSize)]);

if enableFigures
	% Display camera poses.
	camPoses = poses(vSet);
	figure;
	plotCamera(camPoses, 'Size', 0.2);
	hold on
	plotCamera(groundTruthPoses, 'Size', 0.2, 'Color', [0 1 0]);

	xlabel('X');
	ylabel('Y');
	zlabel('Z');

	% Exclude noisy 3-D points.
	goodIdx = (reprojectionErrors < 5);
	xyzPoints = xyzPoints(goodIdx, :);

	% Display the 3-D points.
	pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
		'MarkerSize', 45);
	grid on
	hold off

	% Specify the viewing volume.
	loc1 = camPoses.Location{1};
	xlim([loc1(1)-10, loc1(1)+10]);
	ylim([loc1(2)-6, loc1(2)+6]);
	zlim([loc1(3)-4, loc1(3)+11]);
	camorbit(0, -30);

	title('Refined Camera Poses');
end
