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
rectifiedPosesGT = alignOrientation(rectifiedPosesGT);

% original images
img1 = rgb2gray(imread(fullfile(baseDir, '01.png')));
img2 = rgb2gray(imread(fullfile(baseDir, '02.png')));

%ground truth rectified images
rImg1GT = rgb2gray(imread(fullfile(baseDir, '01_gt.png')));
rImg2GT = rgb2gray(imread(fullfile(baseDir, '02_gt.png')));

loc1 = originalPoses.Location{1};
orient1 = originalPoses.Orientation{1};
loc2 = originalPoses.Location{2};
orient2 = originalPoses.Orientation{2};

[rImg1, rImg2] = rectifyImages(img1, img2, loc1, loc2, orient1, orient2);

%% display results
figure;
imshowSideBySide(img1, img2);
title('Original images');

figure;
imshowSideBySide(rImg1, rImg2);
title('rectified images');

figure;
imshowSideBySide(rImg1GT, rImg2GT);
title('rectified ground truth');