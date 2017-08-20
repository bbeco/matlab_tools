%% Init
clear VARIABLES
imageDir = fullfile('images', 'sfm_test', 'test4', '*.png');
load(fullfile('images', 'sfm_test', 'test4', 'groundTruth.mat'));
% this is the name of the file used to store the results
filename = fullfile('..', 'essentialMatrixTest.xlsx');
imds = imageDatastore(imageDir);

addpath(fullfile('utils'));
addpath(fullfile('coordinate_transform'));
usePointProjection = false;
zMin = 0.037;
dim = 540;
repetitions = 20;

% creating ground truth vectors
[locationGT, orientationGT] = computeRelativeMotion(groundTruthPoses);
orientationGT = orientations2euler(orientationGT);
% converting orientations to degrees and normalizing distances
for i = 1:size(orientationGT, 1)
	orientationGT{i} = orientationGT{i}*180/pi;
	if i ~= 1
		locationGT{i} = locationGT{i}/norm(locationGT{i});
	end
end

% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
	I = readimage(imds, i);
	images{i} = rgb2gray(I);
end

% estimated vectors
locErrorX = cell(repetitions, numel(images));
locErrorY = cell(repetitions, numel(images));
locErrorZ = cell(repetitions, numel(images));
orientErrorX = cell(repetitions, numel(images));
orientErrorY = cell(repetitions, numel(images));
orientErrorZ = cell(repetitions, numel(images));

inliersCounter = zeros(numel(images) - 1, 1);
validPointsFractionCounter = zeros(numel(images) - 1, 1);
pointsForEEstimationCounter = zeros(numel(images) - 1, 1);
pointsForPoseEstimationCounter = zeros(numel(images) - 1, 1);

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

	locErrorX{j, 1} = 0;
	locErrorY{j, 1} = 0;
	locErrorZ{j, 1} = 0;
	orientErrorX{j, 1} = 0;
	orientErrorY{j, 1} = 0;
	orientErrorZ{j, 1} = 0;

	%% remaining images
	for i = 2:numel(images)

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
			iterations, indexPairs, pointsForEEstimationCounter(i - 1), ...
			pointsForPoseEstimationCounter(i - 1)] = ...
			helperEstimateRelativePose(prevConversion, currConversion, ...
			prevFrontIdx, currFrontIdx, indexPairs, cameraParams);

		orientError = abs(rotm2eul(relOrientation)*180/pi - ...
			orientationGT{i});
		orientErrorX{j, i} = orientError(3);
		orientErrorY{j, i} = orientError(2);
		orientErrorZ{j, i} = orientError(1);
		locationError = abs(relLocation - locationGT{i});
		locErrorX{j, i} = locationError(1);
		locErrorY{j, i} = locationError(2);
		locErrorZ{j, i} = locationError(3);

		inliersCounter(i - 1) = sum(inliersIndex);
		validPointsFractionCounter(i - 1) = validPtsFraction;

		% prepare for next image
		prevPoints = currPoints;
		prevFeatures = currFeatures;
		prevConversion = currConversion;
		prevValidIdx = currValidIdx;
		prevFrontIdx = currFrontIdx;
	end
end

%% Write results
groundTruthTable = table(locationGT, orientationGT);
estimatedTable = table(locErrorX);
estimationDataTable = table(inliersCounter, ...
	validPointsFractionCounter, pointsForEEstimationCounter, ...
	pointsForPoseEstimationCounter);

writetable(groundTruthTable, filename, 'Range', 'A2');
writetable(table((1:repetitions)', locErrorX), filename, 'Range', ...
	['A', num2str(numel(images) + 3 + 2)]);
base = numel(images) + 3 + 2;
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