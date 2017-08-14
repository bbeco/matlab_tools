imageDir = fullfile('images', 'sfm_test', 'test4', {'ll0.png', 'll1.png'});
imds = imageDatastore(imageDir);

addpath(fullfile('utils'));
addpath(fullfile('coordinate_transform'));

removeBackPtsBeforeEestimation = false;
zMin = 0.01;

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

% theese indexes have to be re-arranged with the order given by the
% indexPairs vector, then they can be used to select points that belongs to
% the frontal hemisphere before computing the relative camera pose.
frontIdx1 = frontIdx1(indexPairs(:, 1));
frontIdx2 = frontIdx2(indexPairs(:, 2));

if removeBackPtsBeforeEestimation
	indexPairs = indexPairs(frontIdx1 & frontIdx2, :);
end

matchedPts1 = conversion1(indexPairs(:, 1), :);
matchedPts2 = conversion2(indexPairs(:, 2), :);
disp(['Number of matches for E estimation: ', ...
	num2str(size(matchedPts1, 1))]);

for i = 1:100
	% inliersIndex is a logical vector with the points that satisfy the
	% epipolar constraint.
	[E, inliersIndex, status] = estimateEssentialMatrix(matchedPts1, ...
		matchedPts2, cameraParams);

	disp(['inliers points ration: ', num2str(sum(inliersIndex)/numel(inliersIndex))]);

	if status ~= 0
		error('Something is wrong with E estimation');
	end
	
	if sum(inliersIndex) / numel(inliersIndex) < .3
		continue;
	end

	if ~removeBackPtsBeforeEestimation
		% Remove inliers that belongs to the back hemisphere
		inliersIndex = inliersIndex(frontIdx1 & frontIdx2);
	end

	% Valid pointsPtsIndex is the fraction of points that reproject in front of
	% the two cameras it should be above .9
	[relOrientation, relLocation, validPtsIndex] = relativeCameraPose(E, ...
		cameraParams, ...
		matchedPts1(inliersIndex, :), matchedPts2(inliersIndex, :));
	
	if validPtsIndex > .8
		break;
	end
end
relOrientation
relLocation
validPtsIndex