clear;
addpath('geometry');
addpath('image_transform');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('utils');
addpath('ground_truth');
addpath('display_images');

baseDir = fullfile('images/sfm_test/test6');
%resultsDir = fullfile('../results/densification_test/densification5');
load(fullfile(baseDir, 'groundTruth'));
groundTruthPoses = translateLocation(groundTruthPoses);
groundTruthPoses = alignOrientation(groundTruthPoses);

color1 = imread(fullfile(baseDir, 'll1.png'));
color2 = imread(fullfile(baseDir, 'll2.png'));

color1 = imresize(color1, 0.25);
color2 = imresize(color2, 0.25);

[height, width, ~] = size(color1);

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
dm_patchSize = 15;
% disparityList = 1:5:width;
dm_maxDisparity = 180;
dm_metric = 'SSD';
dm_regularization = 0.2;
dm_alpha = 0.05;

disparityRange = [-dm_maxDisparity, dm_maxDisparity];

% ATTENZIONE per maxDisparity quando non e' settata (non sono sicuro
% calcoli il valore corretto dalla GUI
[dispLR, dispRL, maskLR, maskRL] = ...
		computeDisparityEquirectangularCC(im2double(gray1), im2double(gray2), ...
		dm_patchSize, dm_maxDisparity, ...
		dm_metric, dm_regularization, dm_alpha);
% [dispLR, ~] = computeDisparityEquirectangular(gray1, gray2, dm_patchSize, dm_maxDisparity, dm_regularization, dm_alpha);
figure
imshow(mat2gray(abs(dispLR(:,:,1))).*maskLR);
% 
% parfor i = 1:length(regularizationList)
% 	dm_regularization = regularizationList(i);
% 	[dispLR, dispRL, maskLR, maskRL] = ...
% 		computeDisparitySlowCC(im2double(img1), im2double(img2), dm_patchSize, dm_maxDisparity, ...
% 		dm_metric, dm_regularization, dm_alpha);
% 	disparityMapsList{i} = {dispLR, dispRL, maskLR, maskRL};
% 	% load(fullfile(resultsDir, 'workspace.mat'), 'disparityMap');
% % 	figure;
% 	imshow(uint8(255*mat2gray(abs(dispLR(:,:,1)))));
% 	filename = ['disparity_met', dm_metric, '_ps', num2str(dm_patchSize), '_md', ...
% 		num2str(dm_maxDisparity), '_reg', num2str(dm_regularization), ...
% 		'_alpha', num2str(dm_alpha)];
% 	saveas(gcf, fullfile(resultsDir, [filename, '.pdf']));
% 	saveas(gcf, fullfile(resultsDir, [filename, '.fig']));
% end
% 
% %% save workspace
% save(fullfile(resultsDir, 'workspace.mat'));