clear;
baseDir = fullfile('images/densification_test/test1');
resultsDir = fullfile('../results/densification_test/densification4');
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
% dm_maxDisparity = 50;
disparityList = 1:5:height;
dm_metric = 'NCC';
dm_regularization = 0;
dm_alpha = 0;

f = figure;
ax1 = subplot(2, 1, 1);
imshow(img1);
ax2 = subplot(2, 1, 2);
imshow(img2);

if ~exist(fullfile(resultsDir, 'img_pair.pdf'), 'file')
	saveas(gcf, fullfile(resultsDir, 'img_pair.pdf'));
end
if ~exist(fullfile(resultsDir, 'img_pair.fig'), 'file')
	saveas(gcf, fullfile(resultsDir, 'img_pair.fig'));
end

disparityMapsList = cell(1, length(disparityList));
figure;
parfor i = 1:length(disparityList)
	dm_maxDisparity = disparityList(i);
	disparityMapsList{i} = computeDisparityEquirectangular(img1, img2, dm_patchSize, ...
			dm_maxDisparity, dm_regularization, dm_alpha);
	% load(fullfile(resultsDir, 'workspace.mat'), 'disparityMap');
% 	figure;
	imshow(uint8(255*mat2gray(abs(disparityMapsList{i}(:,:,1)))));
	filename = ['disparity_ps', num2str(dm_patchSize), '_md', ...
		num2str(dm_maxDisparity), '_reg', num2str(dm_regularization), ...
		'_alpha', num2str(dm_alpha)];
	saveas(gcf, fullfile(resultsDir, [filename, '.pdf']));
	saveas(gcf, fullfile(resultsDir, [filename, '.fig']));
end

%% save workspace
save(fullfile(resultsDir, 'workspace.mat'));