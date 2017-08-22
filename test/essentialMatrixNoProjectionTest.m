%% Init
clear VARIABLES
imageDir = fullfile('images', 'sfm_test', 'test6', '*.png');
load(fullfile('images', 'sfm_test', 'test6', 'groundTruth.mat'));
% this is the name of the file used to store the results
filename = fullfile('..', 'essentialMatrixTest.xlsx');
imds = imageDatastore(imageDir);

addpath(fullfile('utils'));
addpath(fullfile('coordinate_transform'));
usePointProjection = false;
zMin = 0.037;
dim = 540;
repetitions = 30;

% creating ground truth vectors
[relLocationGT, relOrientationGT] = computeRelativeMotion(groundTruthPoses);
relOrientationGT = orientations2euler(relOrientationGT);
% converting orientations to degrees and normalizing distances
% for i = 1:size(orientationGT, 1)
% 	orientationGT{i} = orientationGT{i}*180/pi;
% 	if i ~= 1
% 		locationGT{i} = locationGT{i}/norm(locationGT{i});
% 	end
% end

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
estimatedX = cell(repetitions, numel(images));
estimatedY = cell(repetitions, numel(images));
estimatedZ = cell(repetitions, numel(images));

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

	locErrorX{j, 1} = 0;
	locErrorY{j, 1} = 0;
	locErrorZ{j, 1} = 0;
	orientErrorX{j, 1} = 0;
	orientErrorY{j, 1} = 0;
	orientErrorZ{j, 1} = 0;
	estimatedX{j, 1} = 0;
	estimatedY{j, 1} = 0;
	estimatedZ{j, 1} = 0;

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
			iterations, indexPairs, pointsForEEstimationCounter(i - 1), ...
			pointsForPoseEstimationCounter(i - 1)] = ...
			helperEstimateRelativePose(prevConversion, currConversion, ...
			prevFrontIdx, currFrontIdx, indexPairs, cameraParams);
		
		scale = norm(relLocationGT{i})/norm(relLocation);

		orientError = abs(rotm2eul(relOrientation) - ...
			relOrientationGT{i})*180/pi;
		orientErrorX{j, i} = orientError(3);
		orientErrorY{j, i} = orientError(2);
		orientErrorZ{j, i} = orientError(1);
		
		estimatedX{j, i} = estimatedX{j, i - 1} + scale*relLocation(1);
		estimatedY{j, i} = estimatedY{j, i - 1} + scale*relLocation(2);
		estimatedZ{j, i} = estimatedZ{j, i - 1} + scale*relLocation(3);
		
		locationError = abs(scale*relLocation - relLocationGT{i});
		locationError = locationError/norm(relLocationGT{i});
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
groundTruthTable = table(relLocationGT, relOrientationGT);
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

writetable(table((1:repetitions)', estimatedX), filename, 'Range', ...
	['A', num2str(6*(repetitions + 3) + base)]);
writetable(table((1:repetitions)', estimatedY), filename, 'Range', ...
	['A', num2str(7*(repetitions + 3) + base)]);
writetable(table((1:repetitions)', estimatedZ), filename, 'Range', ...
	['A', num2str(8*(repetitions + 3) + base)]);

locationGT = cat(1, groundTruthPoses.Location{:});
writetable(table(locationGT(:, 1)'), filename, 'Range', ...
	['A', num2str(9*(repetitions + 3) + base)]);
writetable(table(locationGT(:, 2)'), filename, 'Range', ...
	['A', num2str(10*(repetitions + 3) + base)]);
writetable(table(locationGT(:, 3)'), filename, 'Range', ...
	['A', num2str(11*(repetitions + 3) + base)]);

% writetable(estimationDataTable, filename, 'Range', ...
% 	['A', num2str(9*(repetitions + 3) + base)]);