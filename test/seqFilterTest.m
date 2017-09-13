clear VARIABLES;
addpath('coordinate_transform');
addpath(fullfile('utils'));
addpath(fullfile('filters'));
addpath(fullfile('ground_truth'));
addpath(fullfile('data_analysis'));
% addpath(fullfile('plot'));
dataFolder = fullfile('images', 'sfm_test', 'test10_synthSquare');
% dataFolder = fullfile('images/sfm_test/test8');
imageDir = fullfile(dataFolder, '*.png');
load(fullfile(dataFolder, 'groundTruth.mat'));

% Normalizing ground truth
groundTruthPoses = translateLocation(groundTruthPoses);
groundTruthPoses = alignOrientation(groundTruthPoses);

resultBaseFolder = fullfile('../results/seqFilterTest');

paramTable = table({'name'}, {'file'}, 0, ...
	'VariableNames', {'DataSeriesName', 'OutputFileName', 'AngularThreshold'});
repmat(paramTable, length(0:5:30), 1);
i = 1;
for threshold = 10
	paramTable(i, :) = table(...
		{['threshold ', num2str(threshold)]}, ...
		{fullfile(resultBaseFolder, [num2str(threshold), 'threshold'])}, ...
		threshold, 'VariableNames', ...
		{'DataSeriesName', 'OutputFileName', 'AngularThreshold'} ...
		);
	i = i + 1;
end

paramTable = paramTable(1, :);

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
zMin = 0.04;

prefilterLLKeyPoints = false;
maxLatitudeAngle = 60; %degrees

% Bundle Adjustment parameters
performGlobalBundleAdjustment = false;
bundleAdjustmentAbsoluteTolerance = 1e-05;
bundleAdjustmentRelativeTolerance = 1e-09;
bundleAdjustmentMaxIterations = 300;

% Windowed bundle adjustment parameters
performWindowedBundleAdjustment = false;
windowSize = 2;

% Sequence filter parameters
% ... see experiment params
seqFilterQuantile = 0.8;
% **********************************

imds = imageDatastore(imageDir);
% seqLength = numel(imds.Files);
%The following is the actual number of images processed
seqLength = 35;
pointsForEEstimationCounter = cell(repetitions, size(paramTable, 1));
pointsForPoseEstimationCounter = cell(repetitions, size(paramTable, 1));
frameUsed = cell(repetitions, size(paramTable, 1));
trackSize = cell(repetitions, size(paramTable, 1));
locError = cell(repetitions, size(paramTable, 1));
orientError = cell(repetitions, size(paramTable, 1));
angularXerror = cell(repetitions, size(paramTable, 1));
angularYerror = cell(repetitions, size(paramTable, 1));
angularZerror = cell(repetitions, size(paramTable, 1));

sumAbsLocError = zeros(size(paramTable, 1), 1);
sumAbsLocErrorCI = zeros(size(sumAbsLocError));
sumAbsOrientError = zeros(size(paramTable, 1), 3);
sumAbsOrientErrorCI = zeros(size(sumAbsOrientError));

resultTable = table({zeros(1, seqLength)}, {zeros(1, seqLength)}, {zeros(1, seqLength)}, {zeros(1, seqLength)}, 'VariableNames', {'LocError', 'OrientErrorX', ...
	'OrientErrorY', 'OrientErrorZ'});
resultTable = repmat(resultTable, height(paramTable), 1);
resultCItable = table({zeros(1, seqLength)}, {zeros(1, seqLength)}, {zeros(1, seqLength)}, {zeros(1, seqLength)}, ...
	'VariableNames', {'LocErrorMeanCI', 'OrientErrorXmeanCI', 'OrientErrorYmeanCI', 'OrientErrorZmeanCI'});
resultCItable = repmat(resultCItable, height(paramTable), 1);

for k = 1:height(paramTable)
	% Setting same seed for each experiment
	rng('default');
	
	for i = 1:repetitions

		display(['Experiment: ', num2str(k)]);
		display(['Repetition: ', num2str(i)]);

		[vSet, xyzPoints, ~, ...
			pointsForEEstimationCounter{i, k}, ...
			pointsForPoseEstimationCounter{i, k}, trackSize{i, k}, ...
			frameUsed{i, k}] = ...
			sfmLL_function(imageDir, ...
			computeRelativeScaleBeforeBundleAdjustment, ...
			maxAcceptedReprojectionError, filterMatches, angularThreshold, ...
			zMin, ...
			prefilterLLKeyPoints, maxLatitudeAngle, ...
			performGlobalBundleAdjustment, performWindowedBundleAdjustment, ...
			windowSize, groundTruthPoses, seqLength, ...
			bundleAdjustmentAbsoluteTolerance, ...
			bundleAdjustmentRelativeTolerance, bundleAdjustmentMaxIterations,...
			paramTable.AngularThreshold(k), seqFilterQuantile);
		
		[vSet, ~] = normalizeCameraPosesWithGroundTruth(vSet, ...
			groundTruthPoses(frameUsed{i, k}, :));

		[locError{i, k}, orientError{i, k}] = writeExperimentResultsOnFile(...
			[paramTable.OutputFileName{k}, '_rep', num2str(i), '.xlsx'], ...
			vSet, groundTruthPoses, pointsForEEstimationCounter{i, k}, ...
			pointsForPoseEstimationCounter{i, k}, trackSize{i, k}', frameUsed{i, k}');
		
		if enableFigures
			% Display camera poses.
			camPoses = poses(vSet);
			figure;
			plotCamera(camPoses, 'Size', 0.2);
			hold on
			tmp = groundTruthPoses(frameUsed{i, k}, :);
			tmp.ViewId = camPoses.ViewId;
			plotCamera(tmp, 'Size', 0.2, 'Color', [0 1 0]);

			xlabel('X');
			ylabel('Y');
			zlabel('Z');

			% Exclude noisy 3-D points.
% 			goodIdx = (reprojectionErrors < 5);
% 			xyzPoints = xyzPoints(goodIdx, :);

			% Display the 3-D points.
			pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
				'MarkerSize', 45);
			grid on
			hold off

			% Specify the viewing volume.
			loc1 = camPoses.Location{1};
			xlim([loc1(1)-10, loc1(1)+10]);
			ylim([loc1(2)-6, loc1(2)+6]);
			zlim([loc1(3)-6, loc1(3)+11]);
			camorbit(0, -30);

			title(['Refined Camera Poses ', paramTable.DataSeriesName{k}]);
		end

		angularZerror{i, k} = orientError{i, k}(:, 1);
		angularYerror{i, k} = orientError{i, k}(:, 2);
		angularXerror{i, k} = orientError{i, k}(:, 3);
		
	end
	
	% saving sum of absolute errors
	sumAbsLocError(k) = mean(sum(cat(2, locError{:, k})));
	sumAbsLocErrorCI(k) = computeMeanConfidenceInterval(...
		sum(cat(2, locError{:, k}))');
	sumAbsOrientError(k, 1) = mean(sum(cat(2, angularXerror{:, k})));
	sumAbsOrientErrorCI(k, 1) = ...
		computeMeanConfidenceInterval(sum(cat(2, angularXerror{:, k}))');
	sumAbsOrientError(k, 2) = mean(sum(cat(2, angularYerror{:, k})));
	sumAbsOrientErrorCI(k, 2) = ...
		computeMeanConfidenceInterval(sum(cat(2, angularYerror{:, k}))');
	sumAbsOrientError(k, 3) = mean(sum(cat(2, angularZerror{:, k})));
	sumAbsOrientErrorCI(k, 3) = ...
		computeMeanConfidenceInterval(sum(cat(2, angularZerror{:, k}))');
end

%% sum absolute errors
% figure;
% tmp = sumAbsLocError./sum(cat(1, frameUsed{1, :}), 2);
% errorbar(tmp, sumAbsLocErrorCI);
% title('sum of absolute location errrors');
% axis([0, size(paramTable, 1) + 1, 0, 1]);
% axis 'auto y';
% xlabel('Experiment number');
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsLocError.fig'));
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsLocError.pdf'));
% 
% figure;
% tmp = sumAbsOrientError(:, 1)./sum(cat(1, frameUsed{1, :}), 2);
% errorbar(tmp, sumAbsOrientErrorCI(:, 1));
% title('sum of absolute X orientation errors');
% axis([0, size(paramTable, 1) + 1, 0, 1]);
% axis 'auto y';
% xlabel('Experiment number');
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientXError.fig'));
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientXError.pdf'));
% 
% figure;
% tmp = sumAbsOrientError(:, 2)./sum(cat(1, frameUsed{1, :}), 2);
% errorbar(tmp, sumAbsOrientErrorCI(:, 2));
% title('sum of absolute Y orientation errors');
% axis([0, size(paramTable, 1) + 1, 0, 1]);
% axis 'auto y';
% xlabel('Experiment number');
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientYError.fig'));
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientYError.pdf'));
% 
% figure;
% tmp = sumAbsOrientError(:, 3)./sum(cat(1, frameUsed{1, :}), 2);
% errorbar(tmp, sumAbsOrientErrorCI(:, 3));
% title('sum of absolute Z orientation errors');
% axis([0, size(paramTable, 1) + 1, 0, 1]);
% axis 'auto y';
% xlabel('Experiment number');
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientZError.fig'));
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientZError.pdf'));

%% Saving environment
save(fullfile(resultBaseFolder, 'workspace.mat'));
