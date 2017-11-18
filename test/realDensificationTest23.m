% test for disparity parameters
clear;
addpath('geometry');
addpath('image_transform');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('utils');
addpath('ground_truth');
addpath('display_images');

baseDir = fullfile('images/sfm_test/test23_piazzaLeoni_empoli1');
sfmResults = load(fullfile('../results/realSeqTest/test23/workspace.mat'));

% Sparse point cloud
sparsePoints{1, 1} = sfmResults.xyzPoints;
sparsePoints{1, 2} = uint8([255, 0, 0]);
sparsePoints{1, 2} = repmat(sparsePoints{1, 2}, length(sparsePoints{1, 1}), 1);
resultsDir = fullfile(...
	'../results/realDensificationTest/test23_piazzaLeoni_empoli1');
filename = fullfile(resultsDir, 'sparse.ply');
if exist(filename, 'file') ~= 0
	delete(filename);
end
writePointCloudPLY(sparsePoints{1, 1}, sparsePoints{1, 2}, filename);

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
% lastFrame = 5;

% how many views to skip to form a pair.
viewStep = 4;

% disparity parameters
dm_patchSize = 5;
% disparityList = 1:5:width;
%dm_maxDisparity = 180;
dm_metric = 'SSD';
dm_regularization = 0;
dm_alpha = 0.05;
dm_subtractMeanValue = false;
dm_maxDisparity = -1;
dm_horDisparity = 5;
% densification
minDisp = 5;
scale = 0.25;

maxDistance = 3;

%Result dir
foldername = ['ps', num2str(dm_patchSize), ...
	'_metric', dm_metric, ...
	'_regularization', num2str(dm_regularization), ...
	'_alpha', num2str(dm_alpha), ...
	'_subtractMean', num2str(dm_subtractMeanValue), ...
	'_minDisp', num2str(minDisp), ...
	'_dmMaxDisp', num2str(dm_maxDisparity), ...
	'_maxDistance', num2str(maxDistance), ...
	'_scale', num2str(scale), ...
	'_viewStep', num2str(viewStep)];

resultsDir = fullfile(resultsDir, foldername);
disp(resultsDir);
if exist(resultsDir, 'dir') == 0
	mkdir(resultsDir);
end

statusFileName = fullfile(resultsDir, 'status.mat');
if exist(statusFileName, 'file')
	status = load(statusFileName);
	worldPoints = status.worldPoints;
	firstFrame = status.i + 1;
	cameras = status.cameras;
	dispList = status.dispList;
	xyzPoints = status.xyzPoints;
else
	% this contains N world points stored as an xyz vector and an RGB vector
	worldPoints = cell(1, 2);
	firstFrame = 1;
	cameras = cell(1, 2);
	dispList = cell(size(images, 2) - viewStep, 2);
	xyzPoints = cell(size(images, 2) - viewStep, 2);
end
for i = firstFrame:(lastFrame - viewStep)
	
	loc1 = poses.Location{i};
	orient1 = poses.Orientation{i};

	loc2 = poses.Location{i + viewStep};
	orient2 = poses.Orientation{i + viewStep};

	% This function returns the rotation that has been applied to the first
	% camera (in the pre-multiply form)
	disp(['Rectifying pair: ', num2str(i)]);
	[color1, color2, rot1, rot2] = rectifyImages(images{i}, images{i + viewStep}, ...
		loc1, loc2, orient1, orient2);

	% resizing for performances
	color1 = imresize(color1, scale);
	color2 = imresize(color2, scale);

	gray1 = rgb2gray(color1);
	gray2 = rgb2gray(color2);
% 	gray1 = color1;
% 	gray2 = color2;

	[height, width, ~] = size(color1);

% 	figure(1)
% 	imshow([color1; color2]);
% 	hold on
% 	step = 30;
% 	for col = 1:step:width
% 		line([col, col], [1, 2*height], 'Color', 'r');
% 	end
% 	title('rectified images');
% 	hold off
% 	drawnow;

	if dm_maxDisparity < 0
		[dm_maxDisparity, dm_horDisparity] = computeMaxDisparity(gray1, gray2);
	end

	disparityRange = [-dm_maxDisparity, dm_maxDisparity];

	% ATTENZIONE per maxDisparity quando non e' settata (non sono sicuro
	% calcoli il valore corretto dalla GUI
	if strcmp(dm_metric, 'NCC')
		[dispLR, dispRL, maskLR, maskRL] = ...
				computeDisparityNCCEquirectangularCC(im2double(gray1), im2double(gray2), ...
				dm_patchSize, dm_maxDisparity, dm_horDisparity, ...
				dm_metric, dm_regularization, dm_alpha, dm_subtractMeanValue);
		
		%remove points with low correlation
		tmp = dispLR(:,:,2)>0.5;
		disparityMap = dispLR(:,:,1) .* tmp;
		
	else
		[dispLR, dispRL, maskLR, maskRL] = ...
				computeDisparityEquirectangularCC(im2double(gray1), im2double(gray2), ...
				dm_patchSize, dm_maxDisparity, dm_horDisparity, ...
				dm_metric, dm_regularization, dm_alpha, dm_subtractMeanValue);
		
		disparityMap = dispLR(:,:,1);
	end
	
	dispList{i, 1} = dispLR;
	dispList{i, 2} = maskLR;
	
	figure(2);
	imshow(abs(disparityMap).*maskLR, [0, dm_maxDisparity]);
	colormap(gca, 'jet');
	colorbar;
	drawnow;
	filename = fullfile(resultsDir,[foldername, num2str(i), '.fig']);
	if exist(filename, 'file') ~= 0
		delete(filename);
	end
	saveas(gcf, filename);
	filename = fullfile(resultsDir,[foldername, num2str(i), '.pdf']);
	if exist(filename, 'file') ~= 0
		delete(filename);
	end
	saveas(gcf, filename);
	
	disparityMap = disparityMap .* maskLR;
% 	baseline = norm(loc2 - loc1);
% 	densification
% 	[xyzPoints, colors] = ...
% 		triangulateImagePoints(color1, disparityMap, baseline, minDisp);
	[xyzPoints{i, 1}, xyzPoints{i, 2}] = ...
		myTriangulateMidPoints(color1, disparityMap, ...
		rot1, rot2, ...
		loc1, loc2, orient1, orient2, minDisp, maxDistance);
	
	%compute relative location of cameras
% 	relativeLoc = (loc2 - loc1) * orient1';
	
	filename = fullfile(resultsDir, ['pair', num2str(i), '_', foldername, '.ply']);
	if exist(filename, 'file') ~= 0
		delete(filename);
	end
	writePointCloudPLY(xyzPoints{i, 1}, xyzPoints{i, 2}, filename);
	
	%translating 3D points to the common coordinate system
	for j = 1:size(xyzPoints{i, 1}, 1)
		vec = xyzPoints{i, 1}(j, :)';
% 		vec = orient1' * rot * vec;
		%questa rotazione forse e' da invertire
		vec = orient1' * vec;
		xyzPoints{i, 1}(j, :) = vec' + loc1;
	end
	
	%add points and cameras to the existing set
	worldPoints{1} = [worldPoints{1}; xyzPoints{i, 1}];
	worldPoints{2} = [worldPoints{2}; xyzPoints{i, 2}];
	
	%saving cameras' location
	cameras{1} = [cameras{1}; loc1; loc2];
	cameras{2} = [cameras{2}; 0 255 0; 0 255 0];
	
	% Saving status
	save(statusFileName, 'i', 'worldPoints', 'cameras', 'xyzPoints', 'dispList');
end

filename = fullfile(resultsDir, ['total_points_', foldername, '.ply']);
if exist(filename, 'file') ~= 0
	delete(filename);
end
writePointCloudPLY(worldPoints{1}, worldPoints{2}, filename);

%writing camera positions
filename = fullfile(resultsDir, 'camera.ply');
if exist(filename, 'file') ~= 0
	delete(filename);
end
writePointCloudPLY(cameras{1}, cameras{2}, filename);

filename = fullfile(resultsDir, 'workspace.mat');
if exist(filename, 'file') ~= 0
	delete(filename);
end
save(filename);