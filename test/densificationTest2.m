clear VARIABLES;
baseDir = fullfile('images/densification_test/test1');
resultDir = fullfile('../results/densification_test');
load(fullfile(baseDir, 'poses.mat'));
addpath('geometry');
addpath('image_transform');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('utils');
addpath('ground_truth');
addpath('display_images');

% aligning orientations
% rImg1_small = imread(fullfile(baseDir, 'tsukuba_l.png'));
% rImg2_small = imread(fullfile(baseDir, 'tsukuba_r.png'));
rImg1_small = imread(fullfile(baseDir, 'rImg1_small.jpg'));
rImg2_small = imread(fullfile(baseDir, 'rImg2_small.jpg'));

% resizing for performance
rImg1_small = imresize(rImg1_small, 0.5);
rImg2_small = imresize(rImg2_small, 0.5);

%% display results
% figure;
% imshowSideBySide(img1, img2);
% title('Original images');

figure;
[height, width] = size(rImg1_small);
imshow([verticalLL(rImg1_small) verticalLL(rImg2_small)]);
step = 70;
for row = 1:step:height
	line([1, 2*width], [row, row], 'Color', 'r');
end
title('rectified images');

%% compute disparity
dm_patchSize = 7;
dm_maxDisparity = 80;
dm_metric = 'NCC';
dm_regularization = 0.02;
dm_alpha = 1;

% traditional_disparity = computeDisparitySlow(rImg1_small, rImg2_small, dm_patchSize, dm_maxDisparity, ...
% 	dm_metric, dm_regularization, dm_alpha);
% figure;
% imshow(uint8(255 * mat2gray(traditional_disparity(:, :, 1))));
% save(traditional_disparity, fullfile(resultDir, 'traditional_disparity.mat'));

new_disparity = computeDisparityEquirectangular(rImg1_small, rImg2_small, dm_patchSize, ...
	dm_maxDisparity, dm_regularization);
figure;
imshow(uint8(255 * mat2gray(new_disparity(:, :, 1))));
save(new_disparity, fullfile(resultDir, 'new_disparity.mat'));