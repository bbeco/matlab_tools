imageDir = fullfile('images', 'sfm_test', 'test5', {'ll1.png', 'll2.png'});
imds = imageDatastore(imageDir);

addpath(fullfile('utils'));
addpath(fullfile('coordinate_transform'));
zMin = 0.0878;

% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
	I = readimage(imds, i);
	images{i} = rgb2gray(I);
end

I1 = images{1};
I2 = images{2};

cameraParams = cameraParameters;

[height, width] = size(I1);

points1 = detectSURFFeatures(I1);
points2 = detectSURFFeatures(I2);

features1 = extractFeatures(I1, points1);
features2 = extractFeatures(I2, points2);
% points have to be converted before E estimation, so that we can feed 
% the estimateEssentialMatrix function with the right points (the valid
% onse). Otherwise, it becames more difficult to distinguish between valid
% and invalid points (those points whose z-coordinate is below zMin).
[conversion1, validIdx1, frontIdx1] = createPointsConversionTable(...
	points1, zMin, width, height);
[conversion2, validIdx2, frontIdx2] = createPointsConversionTable(...
	points2, zMin, width, height);

% select those features that represent valid points (whose z-coordinate is
% greater than zMin).
features1 = features1(validIdx1, :);
features2 = features2(validIdx2, :);
indexPairs = matchFeatures(features1, features2, 'Unique', true);

[relOrientation, relLcation, validPtsFraction, iterations] = ...
	helperEstimateRelativePose(points1, points2, ...
	conversion1, conversion2, validIdx1, validIdx2, ...
	frontIdx1, frontIdx2, indexPairs, cameraParams);

disp(['Iterations: ', num2str(iterations)]);
disp('Orientation: ');
rotm2eul(relOrientation)*180/pi
relLocation
validPtsFraction