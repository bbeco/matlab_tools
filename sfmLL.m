%% Init
clear variables;
addpath('utils/');
addpath('filters/');
% imageDir = fullfile('images', 'sfm_test', 'test2', {'ll0.png', 'll1.png', 'll2.png', 'll3.png', 'll4.png'});
imageDir = fullfile('images', 'sfm_test', 'test4', '*.png');
load(fullfile('images', 'sfm_test', 'test4', 'groundTruth.mat'));

% ********** PARAMETERS ************
computeRelativeScaleBeforeBundleAdjustment = true;
maxAcceptedReprojectionError = 5;

% filter those matches whose points have similar coordinates
filterMatches = true;
angularThreshold = 2; %degrees

projectExtractedKeyPointDirections = true;
dim = 270;
f = 1;

prefilterLLKeyPoints = true;
maxLatitudeAngle = 60; %degrees

performBundleAdjustment = true;
% **********************************
	
imds = imageDatastore(imageDir);

% Suppress figure warnings
warning('off', 'images:initSize:adjustingMag');

% Display the images.
% figure
% montage(imds.Files, 'Size', [4, 2]);

% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
    I = readimage(imds, i);
    images{i} = rgb2gray(I);
end

% title('Input Image Sequence');

% projecting parameters
[height, width] = size(images{1});
u0 = dim/2;
v0 = u0;
K = [
	f,	0,	u0;
	0,	f,	v0;
	0,	0,	1;];
cameraParams = cameraParameters('IntrinsicMatrix', K', 'ImageSize', [dim, dim]);

%% Processing first image
% Load first image
I = images{1};

prevPoints = detectSURFFeatures(I);

if prefilterLLKeyPoints
	indexes = filterLLPoints(prevPoints, maxLatitudeAngle, width, height);
	prevPoints = prevPoints(indexes, :);
end

if projectExtractedKeyPointDirections
	[prevPointsConversion, prevLLIndexes] = projectKeyPointDirections(...
		prevPoints, width, height, dim);
else
	[prevPointsConversion, prevLLIndexes] = createPointsConversionTable(...
		prevPoints, width, height, dim);
end

% remove all points that do not fit in image plane or that do not belong to the 
% front projection.
prevPoints = prevPoints(prevLLIndexes, :);
if length(prevPoints) < 8
	error(['Too less key points in front face for image ', num2str(1)]);
end

% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(I, prevPoints);

% Create an empty viewSet object to manage the data associated with each
% view.
vSet = viewSet;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;

vSet = addView(vSet, viewId, 'Points', SURFPoints(prevPointsConversion),...
	'Orientation', eye(3, 'like', prevPoints.Location), 'Location', ...
    zeros(1, 3, 'like', prevPoints.Location));

%% Processing all the other images
for i = 2:numel(images)
    % Undistort the current image.
    I = images{i};

    % Detect, extract and match features.
    currPoints = detectSURFFeatures(I);
	
	if prefilterLLKeyPoints
		indexes = filterLLPoints(currPoints, maxLatitudeAngle, width, height);
		currPoints = currPoints(indexes, :);
	end
	
	if projectExtractedKeyPointDirections
		[currPointsConversion, currLLIndexes] = projectKeyPointDirections(...
			currPoints, width, height, dim);
	else
		[currPointsConversion, currLLIndexes] = createPointsConversionTable(...
			currPoints, width, height, dim);
	end
	
	% Remove all points that does not fit in the front projection image
	currPoints = currPoints(currLLIndexes, :);
	if length(currPoints) < 8
		error(['Too less key points in front face for image ', num2str(i)]);
	end
	
    currFeatures = extractFeatures(I, currPoints);
    indexPairs = matchFeatures(prevFeatures, currFeatures, ...
        'MaxRatio', .7, 'Unique',  true);
	
	if filterMatches
		validIndexes = filterLLMatches(prevPoints, currPoints, ...
			indexPairs, angularThreshold, width, height);
		indexPairs = indexPairs(validIndexes, :);
	end

    % Select matched points.
    projectedMatches1 = SURFPoints(prevPointsConversion(indexPairs(:, 1), :));
    projectedMatches2 = SURFPoints(currPointsConversion(indexPairs(:, 2), :));

    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(...
        projectedMatches1, projectedMatches2, cameraParams);
	
    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', SURFPoints(currPointsConversion));

    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', indexPairs(inlierIdx,:));

    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1);
    prevOrientation = prevPose.Orientation{1};
    prevLocation    = prevPose.Location{1};

    % Compute the current camera pose in the global coordinate system
    % relative to the first view.
    orientation = relativeOrient * prevOrientation;
    location    = prevLocation + relativeLoc * prevOrientation;
	vSet = updateView(vSet, i, 'Orientation', orientation, ...
        'Location', location);
	% From the third image on, compute the relative scale
	if i > 2 && computeRelativeScaleBeforeBundleAdjustment
		relativeScale = computeRelativeScale(vSet, i, cameraParams, maxAcceptedReprojectionError);
		location = prevLocation + relativeScale*relativeLoc*prevOrientation;
		vSet = updateView(vSet, i, 'Location', location);
	end

    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

	% Triangulate initial locations for the 3-D world points.
    [xyzPoints, reprojectionErrors] = triangulateMultiview(tracks, camPoses, cameraParams);
	
	% Triangulate points
% 	[R, t] = cameraPoseToExtrinsics(prevOrientation, prevLocation);
% 	prevCamMatrix = cameraMatrix(cameraParams, R, t);
% 	[R, t] = cameraPoseToExtrinsics(orientation, location);
% 	currCamMatrix = cameraMatrix(cameraParams, R, t);
% 	% I don't select the inlier matched points only. I should try that too, though
% 	[xyzPoints, reprojectionErrors] = triangulate(projectedMatches1, ...
% 		projectedMatches2, prevCamMatrix, currCamMatrix);

    % Refine the 3-D world points and camera poses.
	if performBundleAdjustment
		[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
			tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
			'PointsUndistorted', true);
	end

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
	prevPoints = currPoints;
	prevPointsConversion = currPointsConversion;
end

% Display camera poses.
camPoses = poses(vSet);
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

xlabel('X');
ylabel('Y');
zlabel('Z');

% Exclude noisy 3-D points.
goodIdx = (reprojectionErrors < 5);
xyzPoints = xyzPoints(goodIdx, :);

% Display the 3-D points.
pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-10, loc1(1)+10]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-4, loc1(3)+8]);
camorbit(0, -30);

title('Refined Camera Poses');

%% printing location and orientation result for each view
% computing the absolute scale accorging to ground truth
realDistance = sqrt(sum(groundTruthPoses.Location{2} - groundTruthPoses.Location{1}).^2);
firstPose = poses(vSet, 1);
secondPose = poses(vSet, 2);
estimatedDistance = sqrt(sum(secondPose.Location{1} - firstPose.Location{1}).^2);
absoluteScale = realDistance/estimatedDistance;
for i = 1:vSet.NumViews
	disp(['View ', num2str(i)]);
	disp('Location Error');
	locationError = sqrt(sum(absoluteScale*vSet.Views.Location{i} - ...
		groundTruthPoses.Location{i}).^2);
	disp(locationError);
	disp('Orientation Error');
	orientationError = groundTruthPoses.Orientation{i} - ...
		180/pi*rotm2eul(vSet.Views.Orientation{i});
	disp(orientationError);
end