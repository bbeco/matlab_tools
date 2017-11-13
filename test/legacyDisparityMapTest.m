% test for disparity parameters
clear;
addpath('geometry');
addpath('image_transform');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('utils');
addpath('ground_truth');
addpath('display_images');
addpath('image_transform/');

baseDir = fullfile('images/densification_test/test4/');

img1 = rgb2gray(imread(fullfile(baseDir, 'RectL_vertical.png')));
img2 = rgb2gray(imread(fullfile(baseDir, 'RectR_vertical.png')));

img1 = imresize(img1, 0.25);
img2 = imresize(img2, 0.25);

imshow([img1, img2]);

[height, width, ~] = size(img1);

% disparity parameters
dm_patchSize = 7;
% disparityList = 1:5:width;
%dm_maxDisparity = 180;
dm_metric = 'NCC';
dm_regularization = 0;
dm_alpha = 0.0;
dm_subtractMeanValue = false;
dm_maxDisparity = -1;
dm_horDisparity = 5;

if dm_maxDisparity < 0
	[dm_maxDisparity, ~] = computeMaxDisparity(img1, img2);
end

%Result dir
foldername = ['ps', num2str(dm_patchSize), ...
	'_metric', dm_metric, ...
	'_regularization', num2str(dm_regularization), ...
	'_alpha', num2str(dm_alpha), ...
	'_subtractMean', num2str(dm_subtractMeanValue), ...
	'_dm_maxDisparity', num2str(dm_maxDisparity), ...
	'_dm_horDisparity', num2str(dm_horDisparity)];
resultsDir = fullfile('/tmp/disparityMapTest/', foldername);
if exist(resultsDir, 'dir') == 0
	mkdir(resultsDir);
end

% disparityRange = [-dm_maxDisparity, dm_maxDisparity];

% ATTENZIONE per maxDisparity quando non e' settata (non sono sicuro
% calcoli il valore corretto dalla GUI
if strcmp(dm_metric, 'NCC')
	[dispLR, dispRL, maskLR, maskRL] = ...
			computeDisparitySlowCC(im2double(img1), im2double(img2), ...
			dm_patchSize, dm_maxDisparity, ...
			dm_metric, dm_regularization, dm_alpha);
		
	tmp = dispLR(:,:,2) > 0.5;
	disparityMap = dispLR(:,:,1) .* tmp;
else
	[dispLR, dispRL, maskLR, maskRL] = ...
			computeDisparityEquirectangularCC(im2double(img1), im2double(img2), ...
			dm_patchSize, dm_maxDisparity, dm_horDisparity, ...
			dm_metric, dm_regularization, dm_alpha, dm_subtractMeanValue);
		
	disparityMap = dispLR(:,:,1);
end

figure(1);
imshow(abs(disparityMap).*maskLR, [0, dm_maxDisparity]);
colormap(gca, 'gray');
% colorbar;
drawnow;
filename = fullfile(resultsDir,[foldername, '.fig']);
if exist(filename, 'file') ~= 0
	delete(filename);
end
saveas(gcf, filename);
filename = fullfile(resultsDir,[foldername, '.png']);
if exist(filename, 'file') ~= 0
	delete(filename);
end
saveas(gcf, filename);

filename = fullfile(resultsDir, 'workspace.mat');
if exist(filename, 'file') ~= 0
	delete(filename);
end
save(filename);