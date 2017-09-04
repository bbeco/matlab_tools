clear VARIABLES;
addpath(fullfile('./coordinate_transform'));
addpath(fullfile('./utils/'));
addpath(fullfile('./filters/'));
addpath(fullfile('./ground_truth'));
addpath(fullfile('./data_analysis'));
addpath(fullfile('./plot'));
imageDir = fullfile('./images/sfm_test/test8/*.png');
load(fullfile('./images/sfm_test/test8/groundTruth.mat'));

resultsBaseFolder = fullfile('./../results/bdlAdj_absTolerance_vs_seqLen');
paramTable = table(...
	{'at 1e-02'; 'at 1e-03'; 'at 1e-04'; 'at 1e-05';}, ...
	{fullfile(resultsBaseFolder, '/at_1e-02.xlsx');
	fullfile(resultsBaseFolder, '/at_1e-03.xlsx');
	fullfile(resultsBaseFolder, '/at_1e-04.xlsx'); 
	fullfile(resultsBaseFolder, '/at_1e-05.xlsx')}, ...
	[1e-02; 1e-03; 1e-04; 1e-05], ...
	'VariableNames', {'DataSeriesName', 'OutputFileName', 'AbsoluteTolerance'});

% the sequence length values
seqLength = [3:7, 10, 20];

for i = 1:height(paramTable)
	if exist(paramTable.OutputFileName{i}, 'file')
		delete(paramTable.OutputFileName{i});
	end
end

% ********** PARAMETERS ************
% whether to plot camera position or not
enableFigures = false;
repetitions = 30;

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

% **********************************

imds = imageDatastore(imageDir);
% c = numel(imds.Files);
relLocationError = cell(repetitions, 1);
relOrientationError = cell(repetitions, 1);
pointsForEEstimationCounter = cell(repetitions, 1);
pointsForPoseEstimationCounter = cell(repetitions, 1);
trackSize = cell(repetitions, 1);

sumLocError = zeros(size(paramTable, 1), size(seqLength, 2));
sumLocErrorCI = zeros(size(sumLocError));
sumOrientErrorX = zeros(size(paramTable, 1), size(seqLength, 2));
sumOrientErrorXCI = zeros(size(sumOrientErrorX));
sumOrientErrorY = zeros(size(paramTable, 1), size(seqLength, 2));
sumOrientErrorYCI = zeros(size(sumOrientErrorY));
sumOrientErrorZ = zeros(size(paramTable, 1), size(seqLength, 2));
sumOrientErrorZCI = zeros(size(sumOrientErrorZ));

groundTruthPoses = alignOrientation(groundTruthPoses);

resultTable = table({zeros(1, size(seqLength, 1))}, {zeros(1, size(seqLength, 1))}, {zeros(1, size(seqLength, 1))}, {zeros(1, size(seqLength, 1))}, 'VariableNames', {'LocError', 'OrientErrorX', ...
	'OrientErrorY', 'OrientErrorZ'});
resultTable = repmat(resultTable, height(paramTable), 1);
resultCItable = table({zeros(1, size(seqLength, 1))}, {zeros(1, size(seqLength, 1))}, {zeros(1, size(seqLength, 1))}, {zeros(1, size(seqLength, 1))}, ...
	'VariableNames', {'LocErrorMeanCI', 'OrientErrorXmeanCI', 'OrientErrorYmeanCI', 'OrientErrorZmeanCI'});
resultCItable = repmat(resultCItable, height(paramTable), 1);

for k = 1:height(paramTable)
	% Setting same seed for each experiment
	rng('default');
	index = 1;
	for c = seqLength
	
		locError = zeros(repetitions, c);
		orientError = cell(repetitions, c);
		locXerror = zeros(repetitions, c);
		locYerror = zeros(repetitions, c);
		locZerror = zeros(repetitions, c);
		angularXerror = zeros(repetitions, c);
		angularYerror = zeros(repetitions, c);
		angularZerror = zeros(repetitions, c);

		for i = 1:repetitions

			display(['Experiment: ', num2str(k)]);
			display(['Repetition: ', num2str(i)]);
			display(['Sequence length: ', num2str(c)]);

			[vSet, xyzPoints, reprojectionErrors, ...
				~, ...
				~, ~] = ...
				sfmLL_function(imageDir, ...
				computeRelativeScaleBeforeBundleAdjustment, ...
				maxAcceptedReprojectionError, filterMatches, angularThreshold, ...
				zMin, ...
				prefilterLLKeyPoints, maxLatitudeAngle, ...
				true, false, ...
				2, groundTruthPoses, c, paramTable.AbsoluteTolerance(k));

			[vSet, groundTruthPoses] = normalizeViewSet(vSet, groundTruthPoses);
			camPoses = poses(vSet);

			estLocation = camPoses.Location;
			estOrientation = camPoses.Orientation;
			[tmpLocError, tmpOrientError] = ...
				computePoseError(estLocation, estOrientation, groundTruthPoses, ...
				false, 1:c);

			locError(i, :) = tmpLocError';
			for j = 1:size(camPoses, 1)
				orientError{i, j} =tmpOrientError(j, :);
				angularZerror(i, j) = orientError{i, j}(1);
				angularYerror(i, j) = orientError{i, j}(2);
				angularXerror(i, j) = orientError{i, j}(3);
			end
		end
		sumLocError(k, index) = sum(mean(locError, 1));
		sumOrientErrorX(k, index) = sum(mean(angularXerror, 1));
		sumOrientErrorY(k, index) = sum(mean(angularYerror, 1));
		sumOrientErrorZ(k, index) = sum(mean(angularZerror, 1));
		index = index + 1;
	end

	%% Write results
	filename = paramTable.OutputFileName{k};
	params = table(repetitions, computeRelativeScaleBeforeBundleAdjustment, ...
		maxAcceptedReprojectionError, filterMatches, angularThreshold, ...
		zMin,...
		prefilterLLKeyPoints, maxLatitudeAngle, true, ...
		false, 2);

	writetable(params, filename, 'Range', 'A1');

	tmp = cat(1, groundTruthPoses.Location{:});

	%groundTruth locX
	columnNames = ...
		{'locGTX'};
	groundTruthTable = table(tmp(:, 1)', ...
		'VariableNames', columnNames);
	writetable(groundTruthTable, filename, 'Range', 'A5');

	base = 5;
	tSize = 4;
	tableNumber = 1;

	%groundTruth locY
	columnNames = {'locGTY'};
	errorLocationTable = table(tmp(:, 2)', 'VariableNames', ...
		columnNames);
	writetable(errorLocationTable, filename, 'Range', ...
		['A', num2str(base + tableNumber*tSize)]);

	tableNumber = tableNumber + 1;

	%groundTruth locZ
	columnNames = {'locGTZ'};
	errorLocationTable = table(tmp(:, 3)', 'VariableNames', ...
		columnNames);
	writetable(errorLocationTable, filename, 'Range', ...
		['A', num2str(base + tableNumber*tSize)]);

	tableNumber = tableNumber + 1;

	orientationsGT = orientations2euler(groundTruthPoses.Orientation);
	tmp = cat(1, orientationsGT{:});

	%groundTruth orientX
	columnNames = {'orientGTX'};
	errorLocationTable = table(tmp(:, 3)', 'VariableNames', ...
		columnNames);
	writetable(errorLocationTable, filename, 'Range', ...
		['A', num2str(base + tableNumber*tSize)]);

	tableNumber = tableNumber + 1;

	%groundTruth orientY
	columnNames = {'orientGTY'};
	errorLocationTable = table(tmp(:, 2)', 'VariableNames', ...
		columnNames);
	writetable(errorLocationTable, filename, 'Range', ...
		['A', num2str(base + tableNumber*tSize)]);

	tableNumber = tableNumber + 1;

	%groundTruth orientZ
	columnNames = {'orientGTZ'};
	errorLocationTable = table(tmp(:, 1)', 'VariableNames', ...
		columnNames);
	writetable(errorLocationTable, filename, 'Range', ...
		['A', num2str(base + tableNumber*tSize)]);

	tableNumber = tableNumber + 1;

	base = base + tableNumber*tSize;
	tSize = repetitions + 3;
	tableNumber = 0;
	
% 	%location error
% 	columnNames = {'repetition', 'locError'};
% 	errorLocationTable = table((1:repetitions)', locError, 'VariableNames', ...
% 		columnNames);
% 	writetable(errorLocationTable, filename, 'Range', ...
% 		['A', num2str(base + tableNumber*tSize)]);
% 
% 	tableNumber = tableNumber + 1;
% 
% 	%angular error X
% 	errorXtable = table((1:repetitions)', angularXerror, ...
% 		'VariableNames', {'repetitions', 'orientX_deg'});
% 	writetable(errorXtable, filename, 'Range', ...
% 		['A' num2str(base + tableNumber*tSize)]);
% 
% 	tableNumber = tableNumber + 1;
% 
% 	%angular error Y
% 	errorYtable = table((1:repetitions)', angularYerror, ...
% 		'VariableNames', {'repetitions', 'orientY_deg'});
% 	writetable(errorYtable, filename, 'Range', ...
% 		['A' num2str(base + tableNumber*tSize)]);
% 
% 	tableNumber = tableNumber + 1;
% 
% 	%angular error Z
% 	errorZtable = table((1:repetitions)', angularZerror, ...
% 		'VariableNames', {'repetitions', 'orientZ_deg'});
% 	writetable(errorZtable, filename, 'Range', ...
% 		['A' num2str(base + tableNumber*tSize)]);
% 
% 	tableNumber = tableNumber + 1;
% 
% 	% Estimation data
% 	writetable(...
% 		table(...
% 		(1:repetitions)', pointsForEEstimationCounter, ...
% 		'VariableNames', {'repetitions', 'pointsForEEstimation'}), ...
% 		filename, 'Range', ['A' num2str(base + tableNumber*tSize)]);
% 
% 	tableNumber = tableNumber + 1;
% 
% 	writetable(...
% 		table(...
% 		(1:repetitions)', pointsForPoseEstimationCounter, ...
% 		'VariableNames', {'repetitions', 'pointsForPoseEstimation'}), ...
% 		filename, 'Range', ['A' num2str(base + tableNumber*tSize)]);
% 
% 	tableNumber = tableNumber + 1;
% 
% 	writetable(...
% 		table(...
% 		(1:repetitions)', trackSize, ...
% 		'VariableNames', {'repetitions', 'trackSize'}), ...
% 		filename, 'Range', ['A' num2str(base + tableNumber*tSize)]);
% 
% 	tableNumber = tableNumber + 1;
	
	%% compute mean
	resultTable.LocError{k} = mean(locError, 1);
	resultTable.OrientErrorX{k} = mean(angularXerror, 1);
	resultTable.OrientErrorY{k} = mean(angularYerror, 1);
	resultTable.OrientErrorZ{k} = mean(angularZerror, 1);
	
	resultCItable.LocErrorMeanCI{k} = ...
		computeMeanConfidenceInterval(locError);
	resultCItable.OrientErrorXmeanCI{k} = ...
		computeMeanConfidenceInterval(angularXerror);
	resultCItable.OrientErrorYmeanCI{k} = ...
		computeMeanConfidenceInterval(angularYerror);
	resultCItable.OrientErrorZmeanCI{k} = ...
		computeMeanConfidenceInterval(angularZerror);
	
end

%% Display result
sumLocErrorCI = computeMeanConfidenceInterval(sumLocError);
figure;
% plotWithErrorBar(sumLocError', sumLocErrorCI', paramTable.DataSeriesName);
bar(sumLocError');
legend(paramTable.DataSeriesName);
title('Mean location error');
saveas(gcf, fullfile(resultsBaseFolder, 'locErrorPlot.fig'));
saveas(gcf, fullfile(resultsBaseFolder, 'locErrorPlot.pdf'));

sumOrientErrorXCI = computeMeanConfidenceInterval(sumOrientErrorX);
figure;
% plotWithErrorBar(sumOrientErrorX', sumOrientErrorXCI, paramTable.DataSeriesName);
bar(sumOrientErrorX');
legend(paramTable.DataSeriesName);
title('Mean X orientation error');
saveas(gcf, fullfile(resultsBaseFolder, 'orientErrorXplot.fig'));
saveas(gcf, fullfile(resultsBaseFolder, 'orientErrorXplot.pdf'));

sumOrientErrorYCI = computeMeanConfidenceInterval(sumOrientErrorY);
figure;
% plotWithErrorBar(sumOrientErrorY', sumOrientErrorYCI, paramTable.DataSeriesName);
bar(sumOrientErrorY');
legend(paramTable.DataSeriesName);
title('Mean Y orientation error');
saveas(gcf, fullfile(resultsBaseFolder, 'orientErrorYplot.fig'));
saveas(gcf, fullfile(resultsBaseFolder, 'orientErrorYplot.pdf'));

sumOrientErrorZCI = computeMeanConfidenceInterval(sumOrientErrorZ);
figure;
% plotWithErrorBar(sumOrientErrorZ', sumOrientErrorCI, paramTable.DataSeriesName);
bar(sumOrientErrorZ');
legend(paramTable.DataSeriesName);
title('Mean Z orientation error');
saveas(gcf, fullfile(resultsBaseFolder, 'orientErrorZplot.fig'));
saveas(gcf, fullfile(resultsBaseFolder, 'orientErrorZplot.pdf'));


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
