clear VARIABLES;
addpath('coordinate_transform');
addpath('utils/');
addpath('filters/');
imageDir = fullfile('images', 'sfm_test', 'test4', '*.png');
load(fullfile('images', 'sfm_test', 'test4', 'groundTruth.mat'));
filename = '../test4.xlsx';

% ********** PARAMETERS ************
% whether to plot camera position or not
enableFigures = true;
repetitions = 1;

computeRelativeScaleBeforeBundleAdjustment = false;
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
locXerror = zeros(repetitions, c);
locYerror = zeros(repetitions, c);
locZerror = zeros(repetitions, c);
angularXerror = zeros(repetitions, c);
angularYerror = zeros(repetitions, c);
angularZerror = zeros(repetitions, c);

distanceGT = zeros(c, 1);
orientationGT = zeros(c, 3);

for i = 1:repetitions

	[vSet, xyzPoints, reprojectionErrors] = ...
		sfmLL_function(imageDir, ...
		computeRelativeScaleBeforeBundleAdjustment, ...
		maxAcceptedReprojectionError, filterMatches, angularThreshold, ...
		zMin, ...
		prefilterLLKeyPoints, maxLatitudeAngle, ...
		performBundleAdjustment, viewsWindowSize, groundTruthPoses);

	[vSet, groundTruthPoses] = normalizeViewSet(vSet, groundTruthPoses);
	camPoses = poses(vSet);
	location = cat(1, camPoses.Location{:});
	locationGT = cat(1, groundTruthPoses.Location{:});
	for j = 1:size(camPoses, 1)
		location = camPoses.Location{j};
		locationGT = groundTruthPoses.Location{j};
		locXerror(i, j) = abs(location(1) - locationGT(1));
		locYerror(i, j) = abs(location(2) - locationGT(2));
		locZerror(i, j) = abs(location(3) - locationGT(3));
		
		% rotm2eul expects rotation matrixes in premultiply form, we need to 
		% transpose the orientations before feeding them to rotm2eul.
		orientation = rotm2eul(camPoses.Orientation{j}');
		angularErrors = abs(orientation - ...
			rotm2eul(groundTruthPoses.Orientation{j}'));
		angularZerror(i, j) = angularErrors(1);
		angularYerror(i, j) = angularErrors(2);
		angularXerror(i, j) = angularErrors(3);
	end
end

% Computing distances
for j = 2:c
	distanceGT(j) = abs(sqrt(sum((groundTruthPoses.Location{j} - ...
		groundTruthPoses.Location{j-1}).^2)));
	orientationGT(j, :) = abs(rotm2eul(groundTruthPoses.Orientation{j}') - ...
		rotm2eul(groundTruthPoses.Orientation{j - 1}'));
end

angularXerror = 180/pi * angularXerror;
angularYerror = 180/pi * angularYerror;
angularZerror = 180/pi * angularZerror;
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

%location error X
columnNames = {'repetition', 'locXerror'};
errorLocationTable = table((1:repetitions)', locXerror, ...
	'VariableNames', columnNames);
writetable(errorLocationTable, filename, 'Range', ['A' num2str(base + tSize)]);

%location error Y
columnNames = {'repetition', 'locYerror'};
errorLocationTable = table((1:repetitions)', locYerror, ...
	'VariableNames', columnNames);
writetable(errorLocationTable, filename, 'Range', ['A' num2str(base + 2*tSize)]);

%location error Z
columnNames = {'repetition', 'locZerror'};
errorLocationTable = table((1:repetitions)', locZerror, ...
	'VariableNames', columnNames);
writetable(errorLocationTable, filename, 'Range', ['A' num2str(base + 3*tSize)]);

%angular error X
errorXtable = table((1:repetitions)', angularXerror, ...
	'VariableNames', {'repetitions', 'ErrorX_deg'});
writetable(errorXtable, filename, 'Range', ['A' num2str(base + 4*tSize)]);

%angular error Y
errorYtable = table((1:repetitions)', angularYerror, ...
	'VariableNames', {'repetitions', 'ErrorY_deg'});
writetable(errorYtable, filename, 'Range', ['A' num2str(base + 5*tSize)]);

%angular error Z
errorZtable = table((1:repetitions)', angularZerror, ...
	'VariableNames', {'repetitions', 'ErrorZ_deg'});
writetable(errorZtable, filename, 'Range', ['A' num2str(base + 6*tSize)]);

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
	zlim([loc1(3)-4, loc1(3)+10]);
	camorbit(0, -30);

	title('Refined Camera Poses');
end

% resultsTable = table(mean(mean(errorLocation, 2)), mean(mean(errorX, 2)), ...
% 	mean(mean(errorZ, 2)), ...
% 	'VariableNames', {'AvgErrorLoc', 'AvgErrorX', 'AvgErrorY', 'AvgErrorZ'});
% writetable(resultsTable, filename, 'Range', ['A' num2str(4 + c + 3 + 4*repetitions + 4*3)]);
