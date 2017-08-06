clear VARIABLES;
addpath('coordinate_transform');
addpath('utils/');
addpath('filters/');
imageDir = fullfile('images', 'sfm_test', 'test6', '*.png');
load(fullfile('images', 'sfm_test', 'test6', 'groundTruth.mat'));
filename = '../test6.xlsx';

% ********** PARAMETERS ************
repetitions = 1;

ViewWindowSize = 2;
computeRelativeScaleBeforeBundleAdjustment = false;
maxAcceptedReprojectionError = 0.8;

% filter those matches whose points have similar coordinates
filterMatches = false;
angularThreshold = 2; %degrees

projectExtractedKeyPointDirections = true;
dim = 270;
f = 1;

prefilterLLKeyPoints = false;
maxLatitudeAngle = 60; %degrees

performBundleAdjustment = true;
% **********************************

imds = imageDatastore(imageDir);
c = numel(imds.Files);
errorLocation = zeros(repetitions, c);
errorX = zeros(repetitions, c);
errorY = zeros(repetitions, c);
errorZ = zeros(repetitions, c);

distanceGT = zeros(c, 1);
orientationGT = zeros(c, 3);

params = table(repetitions, computeRelativeScaleBeforeBundleAdjustment, ...
	maxAcceptedReprojectionError, filterMatches, angularThreshold, ...
	projectExtractedKeyPointDirections, dim, f,...
	prefilterLLKeyPoints, maxLatitudeAngle, performBundleAdjustment, ...
	ViewWindowSize);

writetable(params, filename, 'Range', 'A1');

for i = 1:repetitions

	[vSet, xyzPoints, reprojectionErrors] = ...
		sfmLL_function(imageDir, computeRelativeScaleBeforeBundleAdjustment, ...
		maxAcceptedReprojectionError, filterMatches, angularThreshold, ...
		projectExtractedKeyPointDirections, dim, f, ...
		prefilterLLKeyPoints, maxLatitudeAngle, ...
		performBundleAdjustment);

	[vSet, groundTruthPoses] = normalizeViewSet(vSet, groundTruthPoses);
	camPoses = poses(vSet);
	location = cat(1, camPoses.Location{:});
	locationGT = cat(1, groundTruthPoses.Location{:});
	errorLocation = (sqrt(sum((location - locationGT).^2, 2)))';
	for j = 1:size(camPoses, 1)
		location = camPoses.Location{j};
		locationGT = groundTruthPoses.Location{j};
		errorLocation(i, j) = sqrt(sum((location - locationGT).^2));
		
		% rotm2eul expects rotation matrixes in premultiply form, we need to 
		% transpose the orientations before feeding them to rotm2eul.
		orientation = rotm2eul(camPoses.Orientation{j}');
		orientationGT = rotm2eul(groundTruthPoses.Orientation{j}');
		angularErrors = abs(orientation - orientationGT);
		errorZ(i, j) = angularErrors(1);
		errorY(i, j) = angularErrors(2);
		errorX(i, j) = angularErrors(3);
	end
end

%% Write results

% Computing distances
for j = 2:c
	distanceGT(j) = distanceGT(j-1) + ...
		abs(sqrt(sum((groundTruthPoses.Location{j} - groundTruthPoses.Location{j-1}).^2)));
	orientationGT(j, :) = rotm2eul(groundTruthPoses.Orientation{j}');
end

errorX = 180/pi * errorX;
errorY = 180/pi * errorY;
errorZ = 180/pi * errorZ;
orientationGT = 180/pi*orientationGT;

columnNames = ...
	{'distance', 'OrientationX_deg', 'orientationY_deg', 'OrientationZ_deg'};
groundTruthTable = table(distanceGT, ...
	orientationGT(:, 3), orientationGT(:, 2), orientationGT(:, 3), ...
	'VariableNames', columnNames);
writetable(groundTruthTable, filename, 'Range', 'A4');

columnNames = {'repetition', 'LocationErr'};
errorLocationTable = table([1:repetitions]', errorLocation, ...
	'VariableNames', columnNames);
writetable(errorLocationTable, filename, 'Range', ['A' num2str(4 + c + 3)]);
errorXtable = table([1:repetitions]', errorX, ...
	'VariableNames', {'repetitions', 'ErrorX_deg'});
writetable(errorXtable, filename, 'Range', ['A' num2str(4 + c + 2*repetitions + 2*3)]);
errorYtable = table([1:repetitions]', errorY, ...
	'VariableNames', {'repetitions', 'ErrorY_deg'});
writetable(errorYtable, filename, 'Range', ['A' num2str(4 + c + 3*repetitions + 3*3)]);
errorZtable = table([1:repetitions]', errorZ, ...
	'VariableNames', {'repetitions', 'ErrorZ_deg'});
writetable(errorZtable, filename, 'Range', ['A' num2str(4 + c + 4*repetitions + 4*3)]);

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
