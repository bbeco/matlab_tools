clear;
addpath('geometry');
addpath('image_transform');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('utils');
addpath('ground_truth');
addpath('display_images');

baseDir = fullfile('images/densification_test/test3_simple_textured');
resultsDir = fullfile('../results/densification_test/densification7');
load(fullfile(baseDir, 'groundTruth'));
groundTruthPoses = translateLocation(groundTruthPoses);
groundTruthPoses = alignOrientation(groundTruthPoses);

color1 = imread(fullfile(baseDir, 'imgL.png'));
color2 = imread(fullfile(baseDir, 'imgR.png'));

gray1 = rgb2gray(color1);
gray2 = rgb2gray(color2);

loc1 = groundTruthPoses.Location{1};
orient1 = groundTruthPoses.Orientation{1};

loc2 = groundTruthPoses.Location{2};
orient2 = groundTruthPoses.Orientation{2};

[color1, color2] = rectifyImages(color1, color2, ...
	loc1, loc2, orient1, orient2);

[gray1, gray2] = rectifyImages(gray1, gray2, ...
	loc1, loc2, orient1, orient2);

color1 = imresize(color1, 0.5);
color2 = imresize(color2, 0.5);

gray1 = imresize(gray1, 0.5);
gray2 = imresize(gray2, 0.5);

[height, width, ~] = size(color1);

figure
imshow([color1; color2]);
step = 30;
for col = 1:step:width
	line([col, col], [1, 2*height], 'Color', 'r');
end
title('rectified images');
% 
% figure
% imshow([gray1; gray2]);
% for col = 1:step:width
% 	line([col, col], [1, 2*height], 'Color', 'r');
% end
% title('rectified images');

% disparity parameters
dm_patchSize = 9;
% disparityList = 1:5:width;
%dm_maxDisparity = 180;
dm_metric = 'SSD';
dm_regularization = 0;
dm_alpha = 0;

dm_maxDisparity = computeMaxDisparity(gray1, gray2);

disparityRange = [-dm_maxDisparity, dm_maxDisparity];

% ATTENZIONE per maxDisparity quando non e' settata (non sono sicuro
% calcoli il valore corretto dalla GUI
[dispLR, dispRL, maskLR, maskRL] = ...
		computeDisparityEquirectangularCC(im2double(gray1), im2double(gray2), ...
		dm_patchSize, dm_maxDisparity, ...
		dm_metric, dm_regularization, dm_alpha);
figure
imshow(mat2gray(abs(dispLR(:,:,1))).*maskLR);

disparityMap = dispLR.*maskLR;
baseline = norm(loc2 - loc1);
%% densification
%Input data to the densifiction function:
% color1, color2, disparityMap, baseline.
%
%Output by densifiction:
%xyzPoints with color

lambdaMax = 60/180*pi;
xyzPoints = zeros(height*width, 3);
colors = zeros(height*width, 3, 'uint8');
xyzSetLength = 0;

for u = 1:size(disparityMap, 2)
	for v = 1:size(disparityMap, 1)
		% We can not estimate distance without disparity
		if disparityMap(v, u) == 0
			continue;
		end
		
		[latL, long] = extractLLCoordinateFromImage(u, v, width, height);
		[latR, ~] = extractLLCoordinateFromImage(u, v + disparityMap(v, u), ...
			width, height);
		
		%If the points is near the baseline (around the poles' zones in the
		%images, discard it
		if abs(latL) > lambdaMax || abs(latR) > lambdaMax
			continue;
		end
		
		alpha = pi/2 + latL;
		beta = pi/2 + latR;
		
		depth = baseline*(sin(alpha)*sin(beta))/sin(beta - alpha);
		r = depth/sin(alpha);
		
		%computing point's coordinates
		p = r*LL2Cartesian(latL, long);
		
		xyzSetLength = xyzSetLength + 1;
		xyzPoints(xyzSetLength, :) = p;
		
		%extracting color
		colors(xyzSetLength, :) = color1(v, u, :);
	end
end

if exist(fullfile(resultsDir, 'points.ply'), 'file') ~= 0
	delete('points.ply');
end

writePointCloudPLY(xyzPoints, colors, fullfile(resultsDir, 'points.ply'));
save(fullfile(resultsDir, 'workspace'));
saveas(gcf, fullfile(resultsDir, 'disparityMap.fig'));