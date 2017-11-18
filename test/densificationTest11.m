% test for disparity parameters
clear;
addpath('geometry');
addpath('image_transform');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('utils');
addpath('ground_truth');
addpath('display_images');

baseDir = fullfile('images/sfm_test/test6');
load(fullfile(baseDir, 'groundTruth'));
poses = translateLocation(groundTruthPoses);
poses = alignOrientation(poses);

images{1} = imread(fullfile(baseDir, 'll1.png'));
images{2} = imread(fullfile(baseDir, 'll2.png'));
images{3} = imread(fullfile(baseDir, 'll3.png'));

dispList = cell(numel(images) - 1, 2);

% disparity parameters
dm_patchSize = 5;
% disparityList = 1:5:width;
%dm_maxDisparity = 180;
dm_metric = 'SSD';
dm_regularization = 0;
dm_alpha = 0.05;
dm_subtractMeanValue = false;
dm_horDisparity = 0;
dm_maxDisparity = -1;
maxDistance = 10;
minDisp = 3;

%Result dir
foldername = ['ps', num2str(dm_patchSize), ...
	'_metric', dm_metric, ...
	'_regularization', num2str(dm_regularization), ...
	'_alpha', num2str(dm_alpha), ...
	'_subtractMean', num2str(dm_subtractMeanValue)];
resultsDir = fullfile('../results/densification_test/densification11', foldername);
if exist(resultsDir, 'dir') == 0
	mkdir(resultsDir);
end

% this contains N world points stored as an xyz vector and an RGB vector
worldPoints = cell(1, 2);
for i = 1:(numel(images) - 1)
	
	loc1 = poses.Location{i};
	orient1 = poses.Orientation{i};

	loc2 = poses.Location{i + 1};
	orient2 = poses.Orientation{i + 1};

	% This function returns the rotation that has been applied to the first
	% camera (in the pre-multiply form)
	disp(['Rectifying pair: ', num2str(i)]);
	[color1, color2, rot1, rot2] = rectifyImages(images{i}, images{i + 1}, ...
		loc1, loc2, orient1, orient2);

	% resizing for performances
	color1 = imresize(color1, 0.25);
	color2 = imresize(color2, 0.25);

	gray1 = rgb2gray(color1);
	gray2 = rgb2gray(color2);

	[height, width, ~] = size(color1);

	figure
	imshow([color1; color2]);
	hold on
	step = 30;
	for col = 1:step:width
		line([col, col], [1, 2*height], 'Color', 'r');
	end
	title('rectified images');
	hold off

	if dm_maxDisparity < 0
		[dm_maxDisparity, ~] = computeMaxDisparity(gray1, gray2);
	end

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
	baseline = norm(loc2 - loc1);
	% densification
	[xyzPoints, colors] = ...
		myTriangulateMidPoints(color1, disparityMap, ...
		rot1, rot2, ...
		loc1, loc2, orient1, orient2, minDisp, maxDistance);
	
	filename = fullfile(resultsDir, ['pair', num2str(i), '.ply']);
	if exist(filename, 'file') ~= 0
		delete(filename);
	end
	writePointCloudPLY(xyzPoints, colors, filename);
	
	%translating 3D points to the common coordinate system
	for j = 1:size(xyzPoints, 1)
		vec = xyzPoints(j, :)';
		%vec = orient1' * rot * vec;
		vec = orient1' * vec + loc1';
		xyzPoints(j, :) = vec';
	end
	
	%add points to the existing set
	worldPoints{1} = [worldPoints{1}; xyzPoints];
	worldPoints{2} = [worldPoints{2}; colors];
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