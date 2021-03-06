% test for realSeq reconstruction
clear;
addpath('geometry');
addpath('image_transform');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('utils');
addpath('ground_truth');
addpath('display_images');
addpath('filters');

baseDir = fullfile('images/sfm_test/test23_piazzaLeoni_empoli1');
sfmResults = load(fullfile('../results/realSeqSfmTest/test23_piazzaLeoni_empoli1/full_workspace.mat'));
resultsDir = fullfile('../results/realDensificationTest/test23_piazzaLeoni_empoli1/filtered');
if exist(resultsDir, 'dir') == 0
	mkdir(resultsDir);
end

% Sparse point cloud
sparsePoints{1, 1} = sfmResults.xyzPoints;
% sparsePoints{1, 2} = sfmResults.xyzPoints;
sparsePoints{1, 2} = repmat(uint8([255, 0, 0]), length(sparsePoints{1, 1}), 1);

% extracting camera's poses
poses = sfmResults.vSet.poses();
poses = translateLocation(poses);
poses = alignOrientation(poses);

% retriving images
imds = imageDatastore(baseDir);
usedFrames = sfmResults.frameUsed{1, 1};
imagesNumber = find(usedFrames);
images = cell(1, sum(usedFrames));
images = cell(size(images));
for i = 1:length(imagesNumber)
	images{i} = readimage(imds, imagesNumber(i));
end

lastFrame = size(images, 2);
% lastFrame = 3;
maxProcessedViews = 3;

dispList = cell(lastFrame - 1, 2);

% disparity parameters
dm_patchSize = 9;
% disparityList = 1:5:width;
%dm_maxDisparity = 180;
dm_metric = 'SSD';
dm_regularization = 0;
dm_alpha = 0.05;
dm_subtractMeanValue = false;
dm_horDisparity = 0;

% densification
scale = 0.25;
minDisp = 3;

% frame selector parameters
angularThreshold = 30;
quantile = 0.8;

%Result dir
foldername = ['ps', num2str(dm_patchSize), ...
	'_metric', dm_metric, ...
	'_regularization', num2str(dm_regularization), ...
	'_alpha', num2str(dm_alpha), ...
	'_subtractMean', num2str(dm_subtractMeanValue), ...
	'_minDisp', num2str(minDisp), ...
	'_scale', num2str(scale), ...
	'aThreshold', num2str(angularThreshold), ...
	'quantile', num2str(quantile)];
resultsDir = fullfile(resultsDir, foldername);
if exist(resultsDir, 'dir') == 0
	mkdir(resultsDir);
end

% saving sparse cloud
filename = fullfile(resultsDir, 'sparse.ply');
if exist(filename, 'file') ~= 0
	delete(filename);
end
writePointCloudPLY(sparsePoints{1, 1}, sparsePoints{1, 2}, filename);

% this contains N world points stored as an xyz vector and an RGB vector
worldPoints = cell(1, 2);

% first image
prevId = 1;
prevLoc = poses.Location{1};
prevOrient = poses.Orientation{1};
prevColor = images{1};
prevGray = rgb2gray(prevColor);
prevLLPoints = detectSURFFeatures(prevGray);
prevFeatures = extractFeatures(prevGray, prevLLPoints);

currId = 2;
processedImgCounter = 1;
while currId <= (lastFrame - 1)
	
	[height, width, ~] = size(prevColor);
	
	currColor = images{currId};
	currGray = rgb2gray(currColor);
	currLLPoints = detectSURFFeatures(currGray);
	
	% matching
	currFeatures = extractFeatures(currGray, currLLPoints);
	indexPairs = matchFeatures(prevFeatures, currFeatures);
	
	matchedPts1 = prevLLPoints(indexPairs(:, 1), :);
	matchedPts2 = currLLPoints(indexPairs(:, 2), :);
	
	if ~selectFrame(matchedPts1, matchedPts2, angularThreshold, width, height, quantile)
		warning(['pose ', num2str(currId), ' skipped']);
		currId = currId + 1;
		continue;
	end
	
	processedImgCounter = processedImgCounter + 1;

	currLoc = poses.Location{currId};
	currOrient = poses.Orientation{currId};

	% This function returns the rotation that has been applied to the first
	% camera (in the pre-multiply form)
	disp(['Rectifying pair: ', num2str(i)]);
	[color1, color2, rot] = rectifyImages(prevColor, currColor, ...
		prevLoc, currLoc, prevOrient, currOrient);

	% resizing for performances
	color1 = imresize(color1, scale);
	color2 = imresize(color2, scale);

	gray1 = rgb2gray(color1);
	gray2 = rgb2gray(color2);

	[rHeight, rWidth, ~] = size(color1);

	figure
	imshow([color1; color2]);
	hold on
	step = 30;
	for col = 1:step:width
		line([col, col], [1, 2*height], 'Color', 'r');
	end
	title('rectified images');
	hold off

	[dm_maxDisparity, ~] = computeMaxDisparity(gray1, gray2);

	disparityRange = [-dm_maxDisparity, dm_maxDisparity];

	% ATTENZIONE per maxDisparity quando non e' settata (non sono sicuro
	% calcoli il valore corretto dalla GUI
	if strcmp(dm_metric, 'NCC')
		[dispLR, dispRL, maskLR, maskRL] = ...
				computeDisparityNCCEquirectangularCC(im2double(gray1), im2double(gray2), ...
				dm_patchSize, dm_maxDisparity, ...
				dm_metric, dm_regularization, dm_alpha, dm_subtractMeanValue);
	else
		[dispLR, dispRL, maskLR, maskRL] = ...
				computeDisparityEquirectangularCC(im2double(gray1), im2double(gray2), ...
				dm_patchSize, dm_maxDisparity, dm_horDisparity, ...
				dm_metric, dm_regularization, dm_alpha, false);
	end
	
	dispList{i, 1} = dispLR;
	dispList{i, 2} = maskLR;
	
	figure
	imshow(mat2gray(abs(dispLR(:,:,1))).*maskLR);
	filename = fullfile(resultsDir,['disparityMap', num2str(i), '.fig']);
	if exist(filename, 'file') ~= 0
		delete(filename);
	end
	saveas(gcf, filename);
	filename = fullfile(resultsDir,['disparityMap', num2str(i), '.pdf']);
	if exist(filename, 'file') ~= 0
		delete(filename);
	end
	saveas(gcf, filename);
	
	disparityMap = dispLR.*maskLR;
	baseline = norm(currLoc - prevLoc);
	% densification
	[xyzPoints, colors] = ...
		triangulateImagePoints(color1, disparityMap, baseline, minDisp);
	
	filename = fullfile(resultsDir, ['pair', num2str(i), '.ply']);
	if exist(filename, 'file') ~= 0
		delete(filename);
	end
	writePointCloudPLY(xyzPoints, colors, filename);
	
	%translating 3D points to the common coordinate system
	for j = 1:size(xyzPoints, 1)
		vec = xyzPoints(j, :)';
		%ATT. devo tener conto dell'orientazione della prima camera se
		%diversa dall'identita (e' anche vedo che aggiusto le posizioni
		%all'inizio...)
		vec = prevOrient' * rot * vec;
		xyzPoints(j, :) = vec' + prevLoc;
	end
	
	%add points to the existing set
	worldPoints{1} = [worldPoints{1}; xyzPoints];
	worldPoints{2} = [worldPoints{2}; colors];
	
	% preparing variables for the next iteration
	prevLoc = currLoc;
	prevOrient = currOrient;
	prevColor = currColor;
	prevGray = currGray;
	prevLLPoints = currLLPoints;
	prevFeatures = currFeatures;
	currId = currId + 1;
	
	if processedImgCounter >= maxProcessedViews
		break;
	end
end

filename = fullfile(resultsDir, 'total_points.ply');
if exist(filename, 'file') ~= 0
	delete(filename);
end
writePointCloudPLY(worldPoints{1}, worldPoints{2}, filename);

filename = fullfile(resultsDir, 'workspace.mat');
if exist(filename, 'file') ~= 0
	delete(filename);
end
save(filename);
