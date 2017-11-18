% test for disparity parameters
clear;
addpath('geometry');
addpath('image_transform');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('utils');
addpath('ground_truth');
addpath('display_images');

baseDir = fullfile('images/sfm_test/test27_fontana_empoli3');
sfmResults = load(fullfile('../results/realSeqTest/test27/workspace.mat'));

% Sparse point cloud
sparsePoints{1} = sfmResults.xyzPoints{1};
sparsePoints{2} = sfmResults.xyzPoints{2};
resultsDir = fullfile(...
	'../results/realDensificationTest/test27_fontana_empoli3');
filename = fullfile(resultsDir, 'sparse.ply');
if exist(filename, 'file') ~= 0
	delete(filename);
end
writePointCloudPLY(sparsePoints{1}, sparsePoints{2}, filename);

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

% lastFrame = size(images, 2);
lastFrame = 7;

% how many views to skip to form a pair.
viewStep = 3;

dispList = cell(lastFrame - 1, 2);

% disparity parameters
dm_patchSize = 7;
% disparityList = 1:5:width;
%dm_maxDisparity = 180;
dm_metric = 'NCC';
dm_regularization = 0;
dm_alpha = 0.05;
dm_subtractMeanValue = true;
dm_maxDisparity = -1;
dm_horDisparity = 3;
% densification
minDisp = 2;
scale = 0.25;

maxDistance = -1;

qy = 1;
qx = 0.98;

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
else
	% this contains N world points stored as an xyz vector and an RGB vector
	worldPoints = cell(1, 2);
	firstFrame = 1;
	cameras = cell(1, 2);
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

	figure(1)
	imshow([color1; color2]);
	hold on
	step = 30;
	for col = 1:step:width
		line([col, col], [1, 2*height], 'Color', 'r');
	end
	title('rectified images');
	hold off
	drawnow;

	if dm_maxDisparity < 0
		[dm_maxDisparity, ~] = computeMaxDisparity(...
			gray1, gray2, qy, qx);
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
	[xyzPoints, colors] = ...
		myTriangulateMidPoints(color1, disparityMap, ...
		rot1, rot2, ...
		loc1, loc2, orient1, orient2, minDisp, maxDistance);
	
	%compute relative location of cameras
% 	relativeLoc = (loc2 - loc1) * orient1';
	
	filename = fullfile(resultsDir, ['pair', num2str(i), '_', foldername, '.ply']);
	if exist(filename, 'file') ~= 0
		delete(filename);
	end
	writePointCloudPLY(xyzPoints, colors, filename);
	
	%translating 3D points to the common coordinate system
	for j = 1:size(xyzPoints, 1)
		vec = xyzPoints(j, :)';
% 		vec = orient1' * rot * vec;
		vec = orient1' * vec;
		xyzPoints(j, :) = vec' + loc1;
	end
	
	%add points and cameras to the existing set
	worldPoints{1} = [worldPoints{1}; xyzPoints];
	worldPoints{2} = [worldPoints{2}; colors];
	
	%saving cameras' location
	cameras{1} = [cameras{1}; loc1; loc2];
	cameras{2} = [cameras{2}; 0 255 0; 0 255 0];
	
	% Saving status
	save(statusFileName, 'i', 'worldPoints', 'cameras');
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