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
originalPoses = alignOrientation(originalPoses);
% rectifiedPosesGT = alignOrientation(rectifiedPosesGT);

% original images
img1 = rgb2gray(imread(fullfile(baseDir, '01.png')));
img2 = rgb2gray(imread(fullfile(baseDir, '03.png')));

%ground truth rectified images
% rImg1GT = rgb2gray(imread(fullfile(baseDir, '01_gt.png')));
% rImg2GT = rgb2gray(imread(fullfile(baseDir, '02_gt.png')));

loc1 = originalPoses.Location{1};
orient1 = originalPoses.Orientation{1};
loc2 = originalPoses.Location{3};
orient2 = originalPoses.Orientation{3};

[rImg1, rImg2] = rectifyImages(img1, img2, loc1, loc2, orient1, orient2);

%% display results
% figure;
% imshowSideBySide(img1, img2);
% title('Original images');

figure;
rImg1 = verticalLL(rImg1);
rImg2 = verticalLL(rImg2);
[height, width] = size(rImg1);
imshow([rImg1 rImg2]);
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
dm_maxDisparity = -1;
dm_metric = 'NCC';
dm_regularization = 0.02;
dm_alpha = 1;
% disparity = computeDisparitySlow(rImg1, rImg2, dm_patchSize, dm_maxDisparity, ...
% 	dm_metric, dm_regularization, dm_alpha);
disparity = computeDisparityEquirectangular(rImg1, rImg2, dm_maxDisparity, ...
	dm_regularization);

imshow(disparity);