clear VARIABLES;
baseDir = fullfile('images/sfm_test/test6');
load(fullfile(baseDir, 'groundTruth.mat'));
addpath('geometry');
addpath('image_transform');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('utils');
addpath('ground_truth');
addpath('display_images');

% aligning orientations
originalPoses = alignOrientation(groundTruthPoses);
% rectifiedPosesGT = alignOrientation(rectifiedPosesGT);

% original images
img1 = rgb2gray(imread(fullfile(baseDir, 'll2.png')));
img2 = rgb2gray(imread(fullfile(baseDir, 'll3.png')));

[heigth, width] = size(img1);

%ground truth rectified images
% rImg1GT = rgb2gray(imread(fullfile(baseDir, 'rec01_gt.png')));
% rImg2GT = rgb2gray(imread(fullfile(baseDir, 'rec03_gt.png')));

loc1 = originalPoses.Location{2};
orient1 = originalPoses.Orientation{2};
loc2 = originalPoses.Location{3};
orient2 = originalPoses.Orientation{3};

[rImg1, rImg2] = rectifyImages(img1, img2, loc1, loc2, orient1, orient2);

%% display results
% figure;
% imshowSideBySide(img1, img2);
% title('Original images');

figure;
imshow([rImg1; rImg2]);
step = 70;
for col = 1:step:width
	line([col, col], [1, 2*heigth], 'Color', 'r');
end
title('rectified images');

% figure;
% imshow([rImg1GT; rImg2GT]);
% for col = 1:step:width
% 	line([col, col], [1, 2*heigth], 'Color', 'r');
% end
% title('rectified ground truth');