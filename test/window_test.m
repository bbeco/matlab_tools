clear VARIABLES;
addpath('coordinate_transform');
addpath(fullfile('utils'));
addpath(fullfile('filters'));
addpath(fullfile('ground_truth'));
addpath(fullfile('data_analysis'));
addpath(fullfile('plot'));
dataFolder = fullfile('images', 'sfm_test', 'test8');
imageDir = fullfile(dataFolder, '*.png');
load(fullfile(dataFolder, 'groundTruth.mat'));

resultBaseFolder = fullfile('../results/testWindow');
paramTable = table(...
	{'NoBundle'; 'globalBundle'; 'Windowed3+bundle'; 'Windowed4+bundle'; 'Windowed5+bundle';'Windowed6+bundle'}, ...
	{fullfile(resultBaseFolder, 'NoBundle.xlsx');
	fullfile(resultBaseFolder, 'globalBundle.xlsx');
	fullfile(resultBaseFolder, 'windowed3_bundle.xlsx'); 
	fullfile(resultBaseFolder, 'windowed4_bundle.xlsx');
	fullfile(resultBaseFolder, 'windowed5_bundle.xlsx');
	fullfile(resultBaseFolder, 'windowed6_bundle.xlsx')}, ...
	{false; true; true; true; true; true}, {false; false; true; true; true; true}, ...
	[2; 2; 3; 4; 5; 6], ...
	'VariableNames', {'DataSeriesName', 'OutputFileName', 'GlobalBundleAdjustment', ...
	'WindowedBundleAdjustment', 'WindowSize'});

paramTable = paramTable([1, 2, 3], :);

for i = 1:height(paramTable)
	if exist(paramTable.OutputFileName{i}, 'file')
		delete(paramTable.OutputFileName{i});
	end
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
zMin = 0.07;

prefilterLLKeyPoints = false;
maxLatitudeAngle = 60; %degrees

bundleAdjustmentAbsoluteTolerance = 1e-05;

% **********************************

imds = imageDatastore(imageDir);
% c = numel(imds.Files);
%The following is the actual number of images processed
c = 20;
relLocationError = cell(repetitions, 1);
relOrientationError = cell(repetitions, 1);
pointsForEEstimationCounter = cell(repetitions, 1);
pointsForPoseEstimationCounter = cell(repetitions, 1);
trackSize = cell(repetitions, 1);
locError = zeros(repetitions, c);
orientError = cell(repetitions, c);
locXerror = zeros(repetitions, c);
locYerror = zeros(repetitions, c);
locZerror = zeros(repetitions, c);
angularXerror = zeros(repetitions, c);
angularYerror = zeros(repetitions, c);
angularZerror = zeros(repetitions, c);

sumAbsLocError = zeros(size(paramTable, 1), 1);
sumAbsLocErrorCI = zeros(size(sumAbsLocError));
sumAbsOrientError = zeros(size(paramTable, 1), 3);
sumAbsOrientErrorCI = zeros(size(sumAbsOrientError));

groundTruthPoses = alignOrientation(groundTruthPoses);

resultTable = table({zeros(1, c)}, {zeros(1, c)}, {zeros(1, c)}, {zeros(1, c)}, 'VariableNames', {'LocError', 'OrientErrorX', ...
	'OrientErrorY', 'OrientErrorZ'});
resultTable = repmat(resultTable, height(paramTable), 1);
resultCItable = table({zeros(1, c)}, {zeros(1, c)}, {zeros(1, c)}, {zeros(1, c)}, ...
	'VariableNames', {'LocErrorMeanCI', 'OrientErrorXmeanCI', 'OrientErrorYmeanCI', 'OrientErrorZmeanCI'});
resultCItable = repmat(resultCItable, height(paramTable), 1);

for k = 1:height(paramTable)
	% Setting same seed for each experiment
	rng('default');
	
	for i = 1:repetitions

		display(['Experiment: ', num2str(k)]);
		display(['Repetition: ', num2str(i)]);

		[vSet, xyzPoints, reprojectionErrors, ...
			pointsForEEstimationCounter{i}, ...
			pointsForPoseEstimationCounter{i}, trackSize{i}] = ...
			sfmLL_function(imageDir, ...
			computeRelativeScaleBeforeBundleAdjustment, ...
			maxAcceptedReprojectionError, filterMatches, angularThreshold, ...
			zMin, ...
			prefilterLLKeyPoints, maxLatitudeAngle, ...
			paramTable.GlobalBundleAdjustment{k}, paramTable.WindowedBundleAdjustment{k}, ...
			paramTable.WindowSize(k), groundTruthPoses, c, bundleAdjustmentAbsoluteTolerance);

		[vSet, groundTruthPoses] = normalizeViewSet(vSet, groundTruthPoses);
		camPoses = poses(vSet);

		estLocation = camPoses.Location;
		estOrientation = camPoses.Orientation;
		[tmpLocError, tmpOrientError] = ...
			computePoseError(estLocation, estOrientation, groundTruthPoses, ...
			false, 1:c);
		
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

			title(['Refined Camera Poses ', paramTable.DataSeriesName{k}]);
		end

		locError(i, :) = tmpLocError';
		for j = 1:size(camPoses, 1)
			orientError{i, j} =tmpOrientError(j, :);
			angularZerror(i, j) = orientError{i, j}(1);
			angularYerror(i, j) = orientError{i, j}(2);
			angularXerror(i, j) = orientError{i, j}(3);
		end
		
	end
	
	% saving sum of absolute errors
	sumAbsLocError(k) = mean(sum(locError, 2), 1);
	sumAbsLocErrorCI(k) = computeMeanConfidenceInterval(sum(locError, 2));
	sumAbsOrientError(k, 1) = mean(sum(angularXerror, 2), 1);
	sumAbsOrientErrorCI(k, 1) = ...
		computeMeanConfidenceInterval(sum(angularXerror, 2));
	sumAbsOrientError(k, 2) = mean(sum(angularYerror, 2), 1);
	sumAbsOrientErrorCI(k, 2) = ...
		computeMeanConfidenceInterval(sum(angularYerror, 2));
	sumAbsOrientError(k, 3) = mean(sum(angularZerror, 2), 1);
	sumAbsOrientErrorCI(k, 3) = ...
		computeMeanConfidenceInterval(sum(angularZerror, 2));
	

	%% Write results
	filename = paramTable.OutputFileName{k};
	params = table(repetitions, computeRelativeScaleBeforeBundleAdjustment, ...
		maxAcceptedReprojectionError, filterMatches, angularThreshold, ...
		zMin,...
		prefilterLLKeyPoints, maxLatitudeAngle, paramTable.GlobalBundleAdjustment{k}, ...
		paramTable.WindowedBundleAdjustment{k}, paramTable.WindowSize(k));

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

	%location error
	columnNames = {'repetition', 'locError'};
	errorLocationTable = table((1:repetitions)', locError, 'VariableNames', ...
		columnNames);
	writetable(errorLocationTable, filename, 'Range', ...
		['A', num2str(base + tableNumber*tSize)]);

	tableNumber = tableNumber + 1;

	%angular error X
	errorXtable = table((1:repetitions)', angularXerror, ...
		'VariableNames', {'repetitions', 'orientX_deg'});
	writetable(errorXtable, filename, 'Range', ...
		['A' num2str(base + tableNumber*tSize)]);

	tableNumber = tableNumber + 1;

	%angular error Y
	errorYtable = table((1:repetitions)', angularYerror, ...
		'VariableNames', {'repetitions', 'orientY_deg'});
	writetable(errorYtable, filename, 'Range', ...
		['A' num2str(base + tableNumber*tSize)]);

	tableNumber = tableNumber + 1;

	%angular error Z
	errorZtable = table((1:repetitions)', angularZerror, ...
		'VariableNames', {'repetitions', 'orientZ_deg'});
	writetable(errorZtable, filename, 'Range', ...
		['A' num2str(base + tableNumber*tSize)]);

	tableNumber = tableNumber + 1;

	% Estimation data
	writetable(...
		table(...
		(1:repetitions)', pointsForEEstimationCounter, ...
		'VariableNames', {'repetitions', 'pointsForEEstimation'}), ...
		filename, 'Range', ['A' num2str(base + tableNumber*tSize)]);

	tableNumber = tableNumber + 1;

	writetable(...
		table(...
		(1:repetitions)', pointsForPoseEstimationCounter, ...
		'VariableNames', {'repetitions', 'pointsForPoseEstimation'}), ...
		filename, 'Range', ['A' num2str(base + tableNumber*tSize)]);

	tableNumber = tableNumber + 1;

	writetable(...
		table(...
		(1:repetitions)', trackSize, ...
		'VariableNames', {'repetitions', 'trackSize'}), ...
		filename, 'Range', ['A' num2str(base + tableNumber*tSize)]);

	tableNumber = tableNumber + 1;
	
	%% compute mean
	resultTable.LocError{k} = mean(locError, 1);
	resultTable.OrientErrorX{k} = mean(angularXerror, 1);
	resultTable.OrientErrorY{k} = mean(angularYerror, 1);
	resultTable.OrientErrorZ{k} = mean(angularZerror, 1);
	
	resultCItable.LocErrorMeanCI{k} = computeMeanConfidenceInterval(locError);
	resultCItable.OrientErrorXmeanCI{k} = ...
		computeMeanConfidenceInterval(angularXerror);
	resultCItable.OrientErrorYmeanCI{k} = ...
		computeMeanConfidenceInterval(angularYerror);
	resultCItable.OrientErrorZmeanCI{k} = ...
		computeMeanConfidenceInterval(angularZerror);
end

%% Display result
tmpLocErrorResult = cat(1, resultTable.LocError{:});
tmpLocErrorCI = cat(1, resultCItable.LocErrorMeanCI{:});
figure;
plotWithErrorBar(tmpLocErrorResult', tmpLocErrorCI', paramTable.DataSeriesName);
title('Mean location error vs ViewId');
saveas(gcf,fullfile(resultBaseFolder, 'locError.fig'));
saveas(gcf,fullfile(resultBaseFolder, 'locError.pdf'));

tmpOrientErrorXresult = cat(1, resultTable.OrientErrorX{:});
tmpOrientErrorXCI = cat(1, resultCItable.OrientErrorXmeanCI{:});
figure;
plotWithErrorBar(tmpOrientErrorXresult', tmpOrientErrorXCI', paramTable.DataSeriesName);
title('Mean X orientation error vs ViewId');
saveas(gcf,fullfile(resultBaseFolder, 'orientErrorX.fig'));
saveas(gcf,fullfile(resultBaseFolder, 'orientErrorX.pdf'));

tmpOrientErrorYresult = cat(1, resultTable.OrientErrorY{:});
tmpOrientErrorYCI = cat(1, resultCItable.OrientErrorYmeanCI{:});
figure;
plotWithErrorBar(tmpOrientErrorYresult', tmpOrientErrorYCI', paramTable.DataSeriesName);
title('Mean Y orientation error vs ViewId');
saveas(gcf,fullfile(resultBaseFolder, 'orientErrorY.fig'));
saveas(gcf,fullfile(resultBaseFolder, 'orientErrorY.pdf'));

tmpOrientErrorZresult = cat(1, resultTable.OrientErrorZ{:});
tmpOrientErrorZCI = cat(1, resultCItable.OrientErrorZmeanCI{:});
figure;
plotWithErrorBar(tmpOrientErrorZresult', tmpOrientErrorZCI', paramTable.DataSeriesName);
title('Mean Z orientation error vs ViewId');
saveas(gcf,fullfile(resultBaseFolder, 'orientErrorZ.fig'));
saveas(gcf,fullfile(resultBaseFolder, 'orientErrorZ.pdf'));

%% sum absolute errors
figure;
errorbar(sumAbsLocError, sumAbsLocErrorCI);
title('sum of absolute location errrors');
axis([0, size(paramTable, 1) + 1, 0, 1]);
axis 'auto y';
xlabel('Experiment number');
saveas(gcf, fullfile(resultBaseFolder, 'sumAbsLocError.fig'));
saveas(gcf, fullfile(resultBaseFolder, 'sumAbsLocError.pdf'));

figure;
errorbar(sumAbsOrientError(:, 1), sumAbsOrientErrorCI(:, 1));
title('sum of absolute X orientation errors');
axis([0, size(paramTable, 1) + 1, 0, 1]);
axis 'auto y';
xlabel('Experiment number');
saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientXError.fig'));
saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientXError.pdf'));

figure;
errorbar(sumAbsOrientError(:, 2), sumAbsOrientErrorCI(:, 2));
title('sum of absolute Y orientation errors');
axis([0, size(paramTable, 1) + 1, 0, 1]);
axis 'auto y';
xlabel('Experiment number');
saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientYError.fig'));
saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientYError.pdf'));

figure;
errorbar(sumAbsOrientError(:, 3), sumAbsOrientErrorCI(:, 3));
title('sum of absolute Z orientation errors');
axis([0, size(paramTable, 1) + 1, 0, 1]);
axis 'auto y';
xlabel('Experiment number');
saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientZError.fig'));
saveas(gcf, fullfile(resultBaseFolder, 'sumAbsOrientZError.pdf'));

%% Saving environment
save(fullfile(resultBaseFolder, 'workspace.mat'));
