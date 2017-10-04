clear;
baseDir = fullfile('images/densification_test/test1');
load(fullfile(baseDir, 'poses.mat'));
addpath('geometry');
addpath('image_transform');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('utils');
addpath('ground_truth');
addpath('display_images');

img1 = imread(fullfile(baseDir, 'rImg1.jpg'));
img2 = imread(fullfile(baseDir, 'rImg2.jpg'));
img1 = imresize(img1, 0.25);
img2 = imresize(img2, 0.25);
[height, width] = size(img1);

% disparity parameters
dm_patchSize = 7;
dm_maxDisparity = 80;
dm_metric = 'NCC';
dm_regularization = 0.02;
dm_alpha = 1;

figure
subplot(2, 1, 1);
imshow(img1);
ax = subplot(2, 1, 2);
imshow(img2);
hold on

while true
	[x, y, button] = ginput(1);
	if button == 27
		break;
	end
	x = round(x)
	y = round(y)
	[lat, long] = extractLLCoordinateFromImage(x, y, width, height);
	disparityMap = computeDisparityEquirectangular(img1, img2, dm_patchSize, ...
		dm_maxDisparity, dm_regularization);
	plot(ax, x, y + disparityMap(x,y,1), 'r+', 'MarkerSize', 5);
end
