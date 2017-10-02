clear VARIABLES;
baseDir = fullfile('images/densification_test/test1');
load(fullfile(baseDir, 'poses.mat'));
addpath('geometry');
addpath('image_transform');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('utils');
addpath('ground_truth');
addpath('display_images');

% aligning orientations
rImg1_small = imread(fullfile(baseDir, '

%% display results
% figure;
% imshowSideBySide(img1, img2);
% title('Original images');

figure;
[height, width] = size(rImg1);
imshow([verticalLL(rImg1) verticalLL(rImg2)]);
step = 70;
for row = 1:step:height
	line([1, 2*width], [row, row], 'Color', 'r');
end
title('rectified images');

% figure;
% rImg1GT = verticalLL(rImg1GT);
% rImg2GT = verticalLL(rImg2GT);
% imshow([rImg1GT rImg2GT]);
% for row = 1:step:height
% 	line([1, 2*width], [row, row], 'Color', 'r');
% end
% title('rectified ground truth');

%% compute disparity
dm_patchSize = 7;
dm_maxDisparity = 80;
dm_metric = 'NCC';
dm_regularization = 0.02;
dm_alpha = 1;

%scaling the image for performance
rImg1_small = imresize(rImg1, 0.5);
rImg2_small = imresize(rImg2, 0.5);

traditional_disparity = computeDisparitySlow(rImg1_small, rImg2_small, dm_patchSize, dm_maxDisparity, ...
	dm_metric, dm_regularization, dm_alpha);
figure;
imshow(traditional_disparity(:, :, 1));

new_disparity = computeDisparityEquirectangular(rImg1_small, rImg2_small, dm_patchSize, ...
	dm_maxDisparity, dm_regularization);
figure;
imshow(new_disparity(:, :, 1));