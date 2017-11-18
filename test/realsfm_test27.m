clear VARIABLES;
addpath('coordinate_transform');
addpath(fullfile('utils'));
addpath(fullfile('filters'));
addpath(fullfile('ground_truth'));
addpath(fullfile('data_analysis'));
% addpath(fullfile('plot'));
dataFolder = fullfile('images', 'sfm_test', 'test27_fontana_empoli3');
% dataFolder = fullfile('images/sfm_test/test8');
imageDir = fullfile(dataFolder, '*.jpg');
% load(fullfile(dataFolder, 'groundTruth.mat'));

% Normalizing ground truth
% groundTruthPoses = translateLocation(groundTruthPoses);
% groundTruthPoses = alignOrientation(groundTruthPoses);

resultBaseFolder = fullfile('../results/realSeqTest/test27');

paramTable = table({'name'}, {'file'}, 0, 0, ...
	'VariableNames', {'DataSeriesName', 'OutputFileName', 'AngularThreshold',...
	'WindowSize'});
repmat(paramTable, length(0:5:30), 1);
thresholdList = [5];
windowSizeList = [5];
j = 0;
for windowSize = windowSizeList
	i = 1;
	for threshold = thresholdList
		paramTable(j*size(thresholdList, 2) + i, :) = table(...
			{['t_', num2str(threshold), '_W_', num2str(windowSize)]}, ...
			{fullfile(resultBaseFolder, ...
			[num2str(threshold), 'threshold_Window', num2str(windowSize)])}, ...
			threshold, windowSize, 'VariableNames', ...
			{'DataSeriesName', 'OutputFileName', 'AngularThreshold', 'WindowSize'} ...
			);
		i = i + 1;
	end
	j = j + 1;
end

% ********** PARAMETERS ************
% whether to plot camera position or not
enableFigures = true;
repetitions = 1;

computeRelativeScaleBeforeBundleAdjustment = true;
maxAcceptedReprojectionError = 0.8;

% filter those matches whose points have similar coordinates
filterMatches = true;
angularThreshold = 2; %degrees

% minimun threshold for the third components of key points directions.
% This is used when the keypoints are not projected on a new image plane for 
% essential matrix estimation
zMin = 0.04;

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
% windowSize = 2;

% Sequence filter parameters
% ... see experiment params
seqFilterQuantile = 0.8;
% **********************************

imds = imageDatastore(imageDir);
firstFrame = 1;
lastFrame = 400;
% seqLength = numel(imds.Files);
seqLength = lastFrame - firstFrame + 1;
%The following is the actual number of images processed
% seqLength = 121;
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
			paramTable.WindowSize(k), firstFrame, lastFrame, ...
			bundleAdjustmentAbsoluteTolerance, ...
			bundleAdjustmentRelativeTolerance, bundleAdjustmentMaxIterations,...
			paramTable.AngularThreshold(k), seqFilterQuantile);
		
		% setting the same reference for both the estimated poses and ground
		% truth.
% 		[vSet, ~] = normalizeCameraPosesWithGroundTruth(vSet, ...
% 			groundTruthPoses(frameUsed{i, k}, :));

		%% Writing results
% 		[locError{i, k}, orientError{i, k}] = writeExperimentResultsOnFile(...
% 			[paramTable.OutputFileName{k}, '_rep', num2str(i), '.xlsx'], ...
% 			vSet, groundTruthPoses, pointsForEEstimationCounter{i, k}, ...
% 			pointsForPoseEstimationCounter{i, k}, trackSize{i, k}', frameUsed{i, k}');
		
		if enableFigures
			% Display camera poses.
			camPoses = poses(vSet);
			figure;
			plotCamera(camPoses, 'Size', 0.2);
			hold on
% 			tmp = groundTruthPoses(frameUsed{i, k}, :);
% 			tmp.ViewId = camPoses.ViewId;
% 			plotCamera(tmp, 'Size', 0.2, 'Color', [0 1 0]);

			xlabel('X');
			ylabel('Y');
			zlabel('Z');

			% Exclude noisy 3-D points.
% 			goodIdx = (reprojectionErrors < 5);
% 			xyzPoints = xyzPoints(goodIdx, :);

			% Display the 3-D points.
			pcshow(xyzPoints{1}, xyzPoints{2}, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
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
end

for k = 1:height(paramTable)
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
% title('location errror per view');
% axis([0, size(paramTable, 1) + 1, 0, 1]);
% axis 'auto y';
% xlabel('Experiment number');
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsLocError.fig'));
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsLocError.pdf'));
% 
% figure;
% tmp = sumAbsOrientError(:, 1)./sum(cat(1, frameUsed{1, :}), 2);
% errorbar(tmp, sumAbsOrientErrorCI(:, 1));
% title('X orientation error per view');
% axis([0, size(paramTable, 1) + 1, 0, 1]);
% axis 'auto y';
% xlabel('Experiment number');
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientXError.fig'));
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientXError.pdf'));
% 
% figure;
% tmp = sumAbsOrientError(:, 2)./sum(cat(1, frameUsed{1, :}), 2);
% errorbar(tmp, sumAbsOrientErrorCI(:, 2));
% title('Y orientation error per view');
% axis([0, size(paramTable, 1) + 1, 0, 1]);
% axis 'auto y';
% xlabel('Experiment number');
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientYError.fig'));
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientYError.pdf'));
% 
% figure;
% tmp = sumAbsOrientError(:, 3)./sum(cat(1, frameUsed{1, :}), 2);
% errorbar(tmp, sumAbsOrientErrorCI(:, 3));
% title('Z orientation error per view');
% axis([0, size(paramTable, 1) + 1, 0, 1]);
% axis 'auto y';
% xlabel('Experiment number');
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientZError.fig'));
% saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientZError.pdf'));

% Saving sparse point cloud and camera locations
addpath('dense_reconstruction');
camPoses = poses(vSet);
camPoses = cat(1, camPoses.Location{:});
camColors = repmat(uint8([0 255 0]), size(camPoses, 1), 1);
writePointCloudPLY(camPoses, camColors, fullfile(resultBaseFolder, 'camera_location.ply'));
writePointCloudPLY(xyzPoints{1}, xyzPoints{2}, fullfile(resultBaseFolder, 'sparse.ply'));

%% Saving environment
save(fullfile(resultBaseFolder, 'workspace.mat'));
