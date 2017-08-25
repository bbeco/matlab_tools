%% Init
clear VARIABLES
imageDir = fullfile('images', 'sfm_test', 'test9', '*.png');
load(fullfile('images', 'sfm_test', 'test9', 'groundTruth.mat'));
% this is the name of the file used to store the results
filename = fullfile('..', 'essentialMatrixTest.xlsx');
imds = imageDatastore(imageDir);

addpath(fullfile('utils'));
addpath(fullfile('coordinate_transform'));
usePointProjection = false;
zMin = 0.037;
dim = 540;
repetitions = 30;

for i = 2:size(groundTruthPoses, 1)
	groundTruthPoses.Orientation{i} = ...
		groundTruthPoses.Orientation{i}*groundTruthPoses.Orientation{1}';
end
groundTruthPoses.Orientation{1} = eye(3, 'double');

% creating ground truth vectors
[relLocationGT, relOrientationGT] = ...
	computeRelativeMotion(groundTruthPoses);
relativeGT = table((1:size(groundTruthPoses, 1))', relLocationGT, ...
	relOrientationGT, ...
	'VariableNames', {'ViewId', 'Location', 'Orientation'});


% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
	I = readimage(imds, i);
	images{i} = rgb2gray(I);
end

% estimated vectors
location = cell(numel(images), 1);
orientation = cell(numel(images), 1);
locErrorX = cell(repetitions, numel(images));
locErrorY = cell(repetitions, numel(images));
locErrorZ = cell(repetitions, numel(images));
orientErrorX = cell(repetitions, numel(images));
orientErrorY = cell(repetitions, numel(images));
orientErrorZ = cell(repetitions, numel(images));
matches = cell(repetitions, numel(images));
frontInliers = cell(repetitions, numel(images));

if usePointProjection
	K = [1, 0, dim/2;
		0, 1, dim/2;
		0, 0, 1];
	cameraParams = cameraParameters('IntrinsicMatrix', K');
else
	cameraParams = cameraParameters;
end

%% first image
for j = 1:repetitions
	disp(['Repetition: ', num2str(j)]);
	disp('Processing image 1');
	I = images{1};
	[height, width] = size(I);
	prevPoints = detectSURFFeatures(I);

	if usePointProjection
			[prevConversion, prevValidIdx] = ...
				projectKeyPointDirections(prevPoints, width, height, dim);
			prevFrontIdx = ...
				ones(size(prevValidIdx(prevValidIdx)), 'logical');
	else
		% points have to be converted before E estimation, so that we can feed 
		% the estimateEssentialMatrix function with the right points (the valid
		% onse). Otherwise, it becames more difficult to distinguish between valid
		% and invalid points (those points whose z-coordinate is below zMin).
		[prevConversion, prevValidIdx, prevFrontIdx] = createPointsConversionTable(...
			prevPoints, zMin, width, height);
	end

	prevPoints = prevPoints(prevValidIdx, :);
	prevFeatures = extractFeatures(I, prevPoints);
	
	location{1} = [0 0 0];
	orientation{1} = eye(3);

% 	locErrorX{j, 1} = 0;
% 	locErrorY{j, 1} = 0;
% 	locErrorZ{j, 1} = 0;
% 	orientErrorX{j, 1} = 0;
% 	orientErrorY{j, 1} = 0;
% 	orientErrorZ{j, 1} = 0;
% 	estimatedX{j, 1} = 0;
% 	estimatedY{j, 1} = 0;
% 	estimatedZ{j, 1} = 0;

	%% remaining images
	for i = 2:numel(images)
		disp(['Processing image: ', num2str(i)]);
		I = images{i};

		currPoints = detectSURFFeatures(I);

		if usePointProjection
			[currConversion, currValidIdx] = ...
				projectKeyPointDirections(currPoints, width, height, dim);
			currFrontIdx = ...
				ones(size(currValidIdx(currValidIdx)), 'logical');
		else
			[currConversion, currValidIdx, currFrontIdx] = createPointsConversionTable(...
				currPoints, zMin, width, height);
		end

		currPoints = currPoints(currValidIdx, :);
		currFeatures = extractFeatures(I, currPoints);

		% select those features that represent valid points (whose z-coordinate is
		% greater than zMin).
		indexPairs = matchFeatures(prevFeatures, currFeatures, ...
			'MaxRatio', .6, 'Unique', true);

		[relOrientation, relLocation, validPtsFraction, inliersIndex, ...
			iterations, indexPairs, matches{j, i}, frontInliers{j, i}] = ...
			helperEstimateRelativePose(prevConversion, currConversion, ...
			prevFrontIdx, currFrontIdx, indexPairs, cameraParams);
		
		scale = norm(relLocationGT{i})/norm(relLocation);

% 		orientError = abs(rotm2eul(relOrientation) - ...
% 			relOrientationGT{i})*180/pi;
% 		orientErrorX{j, i} = orientError(3);
% 		orientErrorY{j, i} = orientError(2);
% 		orientErrorZ{j, i} = orientError(1);

		location{i} = scale*relLocation;
		orientation{i} = relOrientation;
		
% 		locationError = abs(scale*relLocation - relLocationGT{i});
% 		locationError = locationError/norm(relLocationGT{i});
% 		locErrorX{j, i} = locationError(1);
% 		locErrorY{j, i} = locationError(2);
% 		locErrorZ{j, i} = locationError(3);

		% prepare for next image
		prevPoints = currPoints;
		prevFeatures = currFeatures;
		prevConversion = currConversion;
		prevValidIdx = currValidIdx;
		prevFrontIdx = currFrontIdx;
	end
	
	[locError, orientError] = computePoseError(location, orientation, ...
		relativeGT);
	
	tmp = cat(1, locError{:});
	locErrorX(j, :) = num2cell(tmp(:, 1)');
	locErrorY(j, :) = num2cell(tmp(:, 2)');
	locErrorZ(j, :) = num2cell(tmp(:, 3)');
	tmp = cat(1, orientError{:});
	orientErrorX(j, :) = num2cell(tmp(:, 3)');
	orientErrorY(j, :) = num2cell(tmp(:, 2)');
	orientErrorZ(j, :) = num2cell(tmp(:, 1)');
end

%% Write results
groundTruthTable = table((1:size(relLocationGT, 1))', relLocationGT, ...
	orientations2euler(relOrientationGT), ...
	'VariableNames', {'ViewId', 'Location', 'Orientation'});

estimationDataTable = table((1:repetitions)', matches, frontInliers);

writetable(groundTruthTable, filename, 'Range', 'A2');
writetable(table((1:repetitions)', locErrorX), filename, 'Range', ...
	['A', num2str(size(groundTruthPoses, 1) + 3 + 2)]);
base = size(groundTruthPoses, 1) + 3 + 2;
writetable(table((1:repetitions)', locErrorY), filename, 'Range', ...
	['A', num2str((repetitions + 3) + base)]);
writetable(table((1:repetitions)', locErrorZ), filename, 'Range', ...
	['A', num2str(2*(repetitions + 3) + base)]);
writetable(table((1:repetitions)', orientErrorX), filename, 'Range', ...
	['A', num2str(3*(repetitions + 3) + base)]);
writetable(table((1:repetitions)', orientErrorY), filename, 'Range', ...
	['A', num2str(4*(repetitions + 3) + base)]);
writetable(table((1:repetitions)', orientErrorZ), filename, 'Range', ...
	['A', num2str(5*(repetitions + 3) + base)]);


writetable(estimationDataTable, filename, 'Range', ...
	['A', num2str(6*(repetitions + 3) + base)]);