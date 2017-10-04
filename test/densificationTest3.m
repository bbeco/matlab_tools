clear;
baseDir = fullfile('images/densification_test/test1');
resultsDir = fullfile('../results/densification_test/densification3');
addpath('geometry');
addpath('image_transform');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('utils');
addpath('ground_truth');
addpath('display_images');

img1 = imread(fullfile(baseDir, 'rImg1_small.jpg'));
img2 = imread(fullfile(baseDir, 'rImg2_small.jpg'));

img1 = imresize(img1, 0.5);
img2 = imresize(img2, 0.5);

[height, width] = size(img1);

% disparity parameters
dm_patchSize = 15;
dm_maxDisparity = 100;
dm_metric = 'NCC';
dm_regularization = 0;
dm_alpha = 0;

f = figure;
subplot(2, 1, 1);
imshow(img1);
ax = subplot(2, 1, 2);
imshow(img2);

if ~exist(fullfile(resultsDir, 'img_pair.pdf'), 'file')
	saveas(gcf, fullfile(resultsDir, 'img_pair.pdf'));
end
if ~exist(fullfile(resultsDir, 'img_pair.fig'), 'file')
	saveas(gcf, fullfile(resultsDir, 'img_pair.fig'));
end

disparityMap = computeDisparityEquirectangular(img1, img2, dm_patchSize, ...
		dm_maxDisparity, dm_regularization, dm_alpha);
figure;
imshow(uint8(255*mat2gray(abs(disparityMap(:,:,1)))));
filename = ['disparity_ps', num2str(dm_patchSize), '_md', ...
	num2str(dm_maxDisparity), '_reg', num2str(dm_regularization), ...
	'_alpha', num2str(dm_alpha)];
saveas(gcf, fullfile(resultsDir, [filename, '.pdf']));
saveas(gcf, fullfile(resultsDir, [filename, '.fig']));

%%
figure(f);
while true
	[x, y, button] = ginput(1);
	if button == 27
		break;
	end
	x = round(x)
	y = round(y)
	[lat, long] = extractLLCoordinateFromImage(x, y, width, height);
	plot(ax, x, y + disparityMap(y,x,1), 'r+', 'MarkerSize', 5);
	figure;
end
