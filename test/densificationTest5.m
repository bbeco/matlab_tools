clear;
baseDir = fullfile('images/densification_test/test2');
resultsDir = fullfile('../results/densification_test/densification5');
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

% rotate images
img1 = verticalLL(img1);
img2 = verticalLL(img2);

[height, width] = size(img1);

% disparity parameters
dm_patchSize = 15;
% disparityList = 1:5:width;
dm_maxDisparity = 60;
dm_metric = 'SSD';
% dm_regularization = 0;
regularizationList = [0.01, 0.05:0.05:0.2];
dm_alpha = 0.05;

f = figure;
ax1 = subplot(1, 2, 1);
imshow(img1);
ax2 = subplot(1, 2, 2);
imshow(img2);

if ~exist(fullfile(resultsDir, 'img_pair.pdf'), 'file')
	saveas(gcf, fullfile(resultsDir, 'img_pair.pdf'));
end
if ~exist(fullfile(resultsDir, 'img_pair.fig'), 'file')
	saveas(gcf, fullfile(resultsDir, 'img_pair.fig'));
end

disparityMapsList = cell(length(regularizationList), 4);
figure;
parfor i = 1:length(regularizationList)
	dm_regularization = regularizationList(i);
	[dispLR, dispRL, maskLR, maskRL] = ...
		computeDisparitySlowCC(im2double(img1), im2double(img2), dm_patchSize, dm_maxDisparity, ...
		dm_metric, dm_regularization, dm_alpha);
	disparityMapsList{i} = {{dispLR}, {dispRL}, {maskLR}, {maskRL}};
	% load(fullfile(resultsDir, 'workspace.mat'), 'disparityMap');
% 	figure;
	imshow(uint8(255*mat2gray(abs(dispLR(:,:,1)))));
	filename = ['disparity_met', dm_metric, '_ps', num2str(dm_patchSize), '_md', ...
		num2str(dm_maxDisparity), '_reg', num2str(dm_regularization), ...
		'_alpha', num2str(dm_alpha)];
	saveas(gcf, fullfile(resultsDir, [filename, '.pdf']));
	saveas(gcf, fullfile(resultsDir, [filename, '.fig']));
end

%% save workspace
save(fullfile(resultsDir, 'workspace.mat'));