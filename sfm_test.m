clear VARIABLES;
addpath('coordinate_transform');
addpath('utils/');
addpath('filters/');
addpath('ground_truth');
imageDir = fullfile('images', 'sfm_test', 'test4');
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
[relLocationGT, relOrientationGT] = ...
	computeRelativeMotion(groundTruthPoses);


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
	for j = 1:size(camPoses, 1)
		location = camPoses.Location{j};
		locationGT = groundTruthPoses.Location{j};
		locError{i, j} = norm(camPoses.Location{j} - ...
			groundTruthPoses.Location{j});
		
		orientError{i, j} = 180/pi*abs(...
			rotm2eul(groundTruthPoses.Orientation{j}') -...
			rotm2eul(camPoses.Orientation{j}'));

		angularZerror(i, j) = orientError{i, j}(1);
		angularYerror(i, j) = orientError{i, j}(2);
		angularXerror(i, j) = orientError{i, j}(3);
		
		relLocationError{i, j} = abs(relLocation{j} - relLocationGT{j})/...
			norm(relLocationGT{j});
		relOrientationError{i, j} = abs(rotm2eul(relOrientation{j}') - ...
			rotm2eul(relOrientationGT{j}'))*180/pi;
	end
	
	tmp = cat(1, locError);
	locXerror = tmp(:, 1);
	locYerror = tmp(:, 2);
	locZerror = tmp(:, 3);
end

orientationGT = 180/pi*orientationGT;

%% Write results
params = table(repetitions, computeRelativeScaleBeforeBundleAdjustment, ...
	maxAcceptedReprojectionError, filterMatches, angularThreshold, ...
	zMin,...
	prefilterLLKeyPoints, maxLatitudeAngle, performBundleAdjustment, ...
	viewsWindowSize);

writetable(params, filename, 'Range', 'A1');

columnNames = ...
	{'distance', 'OrientationX_deg', 'orientationY_deg', 'OrientationZ_deg'};
groundTruthTable = table(distanceGT, ...
	orientationGT(:, 3), orientationGT(:, 2), orientationGT(:, 3), ...
	'VariableNames', columnNames);
writetable(groundTruthTable, filename, 'Range', 'A4');

base = 4 + c;
tSize = repetitions + 3;
tableNumber = 1;

%location error
columnNames = {'repetition', 'locError'};
errorLocationTable = table((1:repetitions)', locError, 'VariableNames', ...
	columnNames);
writetable(errorLocationTable, filename, 'Range', ...
	['A', num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

%angular error X
errorXtable = table((1:repetitions)', angularXerror, ...
	'VariableNames', {'repetitions', 'ErrorX_deg'});
writetable(errorXtable, filename, 'Range', ...
	['A' num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

%angular error Y
errorYtable = table((1:repetitions)', angularYerror, ...
	'VariableNames', {'repetitions', 'ErrorY_deg'});
writetable(errorYtable, filename, 'Range', ...
	['A' num2str(base + tableNumber*tSize)]);

tableNumber = tableNumber + 1;

%angular error Z
errorZtable = table((1:repetitions)', angularZerror, ...
	'VariableNames', {'repetitions', 'ErrorZ_deg'});
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
