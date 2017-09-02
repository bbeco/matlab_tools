%% Init
clear VARIABLES

imageDir = fullfile('images', 'sfm_test', 'test4', '*.png');
load(fullfile('images', 'sfm_test', 'test4', 'groundTruth.mat'));
% this is the name of the file used to store the results
imds = imageDatastore(imageDir);

addpath(fullfile('utils'));
addpath(fullfile('coordinate_transform'));
addpath(fullfile('ground_truth'));
addpath(fullfile('data_analysis'));
addpath(fullfile('plot'));
usePointProjection = false;
zMin = 0.037;
dim = 540;
repetitions = 30;

groundTruthPoses = alignOrientation(groundTruthPoses);

% creating ground truth vectors
relativeGT = computeRelativeMotion(groundTruthPoses);


% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
	I = readimage(imds, i);
	images{i} = rgb2gray(I);
end

if usePointProjection
	K = [1, 0, dim/2;
		0, 1, dim/2;
		0, 0, 1];
	cameraParams = cameraParameters('IntrinsicMatrix', K');
else
	cameraParams = cameraParameters;
end

resultBaseFolder = fullfile('..', 'results', 'essentialMatrixEstimationTest');
expParam = table(...
	{'singleEval 150e03trials'; 'singleEval 5e06trials'; 'loopEval 50e03trials'}, ...
	{fullfile('..', 'results', 'essentialMatrixEstimationTest', 'singleEval_150000trials.xlsx');
	fullfile('..', 'results', 'essentialMatrixEstimationTest', 'singleEval_5000000trials.xlsx');
	fullfile('..', 'results', 'essentialMatrixEstimationTest', 'loopEval_50000trials.xlsx')}, ...
	{fullfile('..', 'results', 'essentialMatrixEstimationTest', 'singleEval_150000trials.csv');
	fullfile('..', 'results', 'essentialMatrixEstimationTest', 'singleEval_5000000trials.csv');
	fullfile('..', 'results', 'essentialMatrixEstimationTest', 'loopEval_50000trials.cvs')}, ...
	[150000; 5000000; 50000], ...
	[1; 1; 100], ...
	'VariableNames', {'ExperimentName', 'SheetFileName', 'CsvFileName', 'MaxNumTrials', 'MaxIterations'});

meanLocationError = zeros(size(expParam, 1), numel(images));
meanLocationCI = zeros(size(meanLocationError));
meanOrientationErrorX = zeros(size(expParam, 1), numel(images));
meanOrientationCIX = zeros(size(meanOrientationErrorX));
meanOrientationErrorY = zeros(size(expParam, 1), numel(images));
meanOrientationCIY = zeros(size(meanOrientationErrorY));
meanOrientationErrorZ = zeros(size(expParam, 1), numel(images));
meanOrientationCIZ = zeros(size(meanOrientationErrorZ));

for k = 1:size(expParam, 1)
	%setting seed
	rng('default');
	
	% estimated vectors
	location = cell(numel(images), 1);
	orientation = cell(numel(images), 1);
	locError = zeros(repetitions, numel(images));
	orientErrorX = zeros(repetitions, numel(images));
	orientErrorY = zeros(repetitions, numel(images));
	orientErrorZ = zeros(repetitions, numel(images));
	matches = cell(repetitions, numel(images));
	frontInliers = cell(repetitions, numel(images));
	validPtsFraction = cell(repetitions, numel(images));
	
	%% first image
	for j = 1:repetitions
		disp(['Experiment: ', num2str(k)]);
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

			[relOrientation, relLocation, validPtsFraction{j, i}, inliersIndex, ...
				iterations, indexPairs, matches{j, i}, frontInliers{j, i}] = ...
				helperEstimateRelativePose(prevConversion, currConversion, ...
				prevFrontIdx, currFrontIdx, indexPairs, cameraParams, expParam.MaxNumTrials(k), expParam.MaxIterations(k));

			scale = computeRelativeScaleFromGroundTruth(...
				groundTruthPoses, relLocation, i, i - 1);

			location{i} = scale*relLocation;
			orientation{i} = relOrientation;

			% prepare for next image
			prevPoints = currPoints;
			prevFeatures = currFeatures;
			prevConversion = currConversion;
			prevValidIdx = currValidIdx;
			prevFrontIdx = currFrontIdx;
		end

		% This computes the relative error
		[le, orientError] = computePoseError(location, orientation, ...
			relativeGT, true);

		locError(j, :) = le';
		orientErrorX(j, :) = orientError(:, 3)';
		orientErrorY(j, :) = orientError(:, 2)';
		orientErrorZ(j, :) = orientError(:, 1)';
	end

	%% Write results
	filename = expParam.SheetFileName{k};
	len = size(relativeGT.Orientation, 1);
	relativeGTOrientationDegrees = cell(len, 1);
	for i = 1:len
		relativeGTOrientationDegrees{i} = 180/pi*rotm2eul(relativeGT.Orientation{i}');
	end

	groundTruthTable = table(relativeGT.ViewId, relativeGT.Location, ...
		relativeGTOrientationDegrees, ...
		'VariableNames', {'ViewId', 'Location', 'Orientation'});

	writetable(groundTruthTable, filename, 'Range', 'A2');
	writetable(table((1:repetitions)', locError), filename, 'Range', ...
		['A', num2str(size(groundTruthPoses, 1) + 3 + 2)]);
	base = size(groundTruthPoses, 1) + 3 + 2;
	numTable = 1;

	writetable(...
		table(mean(locError), 'VariableNames', {'LocErrorMean'}), filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;

	writetable(...
		table(computeMeanConfidenceInterval(locError), ...
		'VariableNames', {'LocErrorMeanCI'}), filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;

	writetable(table((1:repetitions)', orientErrorX), filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;

	writetable(...
		table(mean(orientErrorX), 'VariableNames', {'OrientErrorMean'}), filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;

	writetable(...
		table(computeMeanConfidenceInterval(orientErrorX), ...
		'VariableNames', {'orientErrorXMeanCI'}), filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;

	writetable(table((1:repetitions)', orientErrorY), filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;

	writetable(...
		table(mean(orientErrorY), 'VariableNames', {'OrientErrorYMean'}), filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;

	writetable(...
		table(computeMeanConfidenceInterval(orientErrorY), ...
		'VariableNames', {'orientErrorYMeanCI'}), filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;

	writetable(table((1:repetitions)', orientErrorZ), filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;

	writetable(...
		table(mean(orientErrorZ), 'VariableNames', {'orientErrorZMean'}), filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;

	writetable(...
		table(computeMeanConfidenceInterval(orientErrorZ), ...
		'VariableNames', {'orientErrorMeanCI'}), filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;


	estimationDataTable = table((1:repetitions)', matches);
	writetable(estimationDataTable, filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;

	estimationDataTable = table((1:repetitions)', frontInliers);
	writetable(estimationDataTable, filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;

	estimationDataTable = table((1:repetitions)', validPtsFraction);
	writetable(estimationDataTable, filename, 'Range', ...
		['A', num2str(numTable*(repetitions + 3) + base)]);
	numTable = numTable + 1;
	
	meanLocationError(k, :) = mean(locError, 1);
	meanOrientationErrorX(k, :) = mean(orientErrorX, 1);
	meanOrientationErrorY(k, :) = mean(orientErrorY, 1);
	meanOrientationErrorZ(k, :) = mean(orientErrorZ, 1);
	meanLocationCI(k, :) = computeMeanConfidenceInterval(locError);
	meanOrientationCIX(k, :) = computeMeanConfidenceInterval(orientErrorX);
	meanOrientationCIY(k, :) = computeMeanConfidenceInterval(orientErrorY);
	meanOrientationCIZ(k, :) = computeMeanConfidenceInterval(orientErrorZ);
end
%% save plots
figure
plotWithErrorBar(meanLocationError', meanLocationCI', expParam.ExperimentName);
title('Mean Location Error');
saveas(gcf, [resultBaseFolder, '/meanLocationError.fig']);
saveas(gcf, [resultBaseFolder, '/meanLocationError.pdf']);

figure
plotWithErrorBar(meanOrientationErrorX', meanOrientationCIX', expParam.ExperimentName);
saveas(gcf, [resultBaseFolder, '/meanOrientErrorX.fig']);
saveas(gcf, [resultBaseFolder, '/meanOrientErrorX.pdf']);
title('Mean orientation error aroung X');

figure
plotWithErrorBar(meanOrientationErrorY', meanOrientationCIY', expParam.ExperimentName);
saveas(gcf, [resultBaseFolder, '/meanOrientErrorY.fig']);
saveas(gcf, [resultBaseFolder, '/meanOrientErrorY.pdf']);
title('Mean orientation error aroung Y');

figure
plotWithErrorBar(meanOrientationErrorZ', meanOrientationCIZ', expParam.ExperimentName);
saveas(gcf, [resultBaseFolder, '/meanOrientErrorZ.fig']);
saveas(gcf, [resultBaseFolder, '/meanOrientErrorZ.pdf']);
title('Mean orientation error aroung Z');