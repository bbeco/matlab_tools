clear VARIABLES;
addpath('coordinate_transform');
addpath(fullfile('utils'));
addpath(fullfile('filters'));
addpath(fullfile('ground_truth'));
addpath(fullfile('data_analysis'));
% addpath(fullfile('plot'));
dataFolder = fullfile('/home/andrea/seqTest2/frames');
% dataFolder = fullfile('images/sfm_test/test8');
imageDir = fullfile(dataFolder, '*.jpg');

resultBaseFolder = fullfile('../results/realSeqTest');

paramTable = table({'name'}, {'file'}, 0, 0, ...
	'VariableNames', {'DataSeriesName', 'OutputFileName', 'AngularThreshold',...
	'WindowSize'});
repmat(paramTable, length(0:5:30), 1);
thresholdList = [6];
windowSizeList = [5];
j = 0;
for windowSize = windowSizeList
	i = 1;
	for threshold = thresholdList
		paramTable(j*size(thresholdList, 2) + i, :) = table(...
			{['t', num2str(threshold), ' W', num2str(windowSize)]}, ...
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
filterMatches = false;
angularThreshold = 2; %degrees

% minimun threshold for the third components of key points directions.
% This is used when the keypoints are not projected on a new image plane for 
% essential matrix estimation
zMin = 0.04;

prefilterLLKeyPoints = false;
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
% seqLength = numel(imds.Files);
%The following is the actual number of images processed
seqLength = 583;
pointsForEEstimationCounter = cell(repetitions, size(paramTable, 1));
pointsForPoseEstimationCounter = cell(repetitions, size(paramTable, 1));
frameUsed = cell(repetitions, size(paramTable, 1));
trackSize = cell(repetitions, size(paramTable, 1));

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
			paramTable.WindowSize(k), seqLength, ...
			bundleAdjustmentAbsoluteTolerance, ...
			bundleAdjustmentRelativeTolerance, bundleAdjustmentMaxIterations,...
			paramTable.AngularThreshold(k), seqFilterQuantile);
		
		%% Plot camera poses
		if enableFigures
			% Display camera poses.
			camPoses = poses(vSet);
			figure;
			plotCamera(camPoses, 'Size', 0.2);
			hold on

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
			saveas(gcf, ...
				[paramTable.OutputFileName{k}, '_rep', num2str(i),  '.fig']);
			
			%% Writing results
			filename = [paramTable.OutputFileName{k}, '.xlsx'];
			locations = cat(1, vSet.Views.Location{:});
			orientations = zeros(vSet.NumViews, 3);
			for j = 1:size(orientations, 1)
				orientations(j, :) = 180/pi*rotm2eul(vSet.Views.Orientation{j});
			end
			writetable(table(vSet.Views.ViewId, locations, orientations, ...
				'VariableNames', {'ViewId', 'Location', 'Orientation'}), ...
				filename, 'Range', 'A4');
			base = 4 + 3 + vSet.NumViews;
			
			writetable(table( ...
				(1:vSet.NumViews)', ...
				pointsForEEstimationCounter{i, k}(frameUsed{i, k}), ...
				'VariableNames', {'ViewId', 'PointsForEEstimation'}), ...
				filename, 'Range', ...
				['A', num2str(base)]);
			base = base + 3 + vSet.NumViews;
			
			writetable(table(...
				(1:vSet.NumViews)', ...
				pointsForPoseEstimationCounter{i, k}(frameUsed{i, k}), ...
				'VariableNames', {'ViewId', 'PointsForEEstimation'}), ...
				filename, 'Range', ...
				['A', num2str(base)]);
			base = base + 3 + vSet.NumViews;
			
			writetable(table(...
				(1:vSet.NumViews)', ...
				trackSize{i, k}(frameUsed{i, k})', ...
				'VariableNames', {'ViewId', 'trackSize'}), ...
				filename, 'Range', ...
				['A', num2str(base)]);
			base = base + 3 + vSet.NumViews;
			
			writetable(table(...
				(1:seqLength)', ...
				frameUsed{i, k}', ...
				'VariableNames', {'frameNum', 'frameUsed'}), ...
				filename, 'Range', ...
				['A', num2str(base)]);
		end
		
	end
end

%% Saving environment
save(fullfile(resultBaseFolder, 'workspace.mat'));
