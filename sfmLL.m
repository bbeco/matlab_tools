%% Init
clear variables;
addpath('utils/');
% imageDir = fullfile('images', 'essential_matrix_test', {'ll0.png', 'll1.png', 'll2.png', 'll3.png', 'll4.png', 'll5.png', 'll6.png'});
imageDir = fullfile('images', 'essential_matrix_test', {'ll0.png', 'll1.png'});
	
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
dim = min(height, width);
f = 1;
u0 = dim/2;
v0 = u0;
K = [
	f,	0,	u0;
	0,	f,	v0;
	0,	0,	1;];
cameraParams = cameraParameters('IntrinsicMatrix', K');

%% Processing first image
% Load first image
I = images{1};

prevPoints = detectSURFFeatures(I);

[prevPointsConversion, prevLLIndexes] = createPointsConversionTable(...
	prevPoints, width, height, dim);
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

vSet = addView(vSet, viewId, 'Points', SURFPoints(prevPointsConversion),'Orientation', ...
    eye(3, 'like', prevPoints.Location), 'Location', ...
    zeros(1, 3, 'like', prevPoints.Location));

%% Processing second image
% load second image
I = images{2};

viewId = 2;

% Detect, extract and match features.
currPoints = detectSURFFeatures(I);

[currPointsConversion, currLLIndexes] = createPointsConversionTable(...
	currPoints, width, height, dim);
% Remove all points that does not fit in the front projection image
currPoints = currPoints(currLLIndexes, :);

currFeatures = extractFeatures(I, currPoints);

indexPairs = matchFeatures(prevFeatures, currFeatures, ...
	'Unique', true);
% [prevPointsConversion, prevLLIndexes] = createPointsConversionTable(...
% 	prevPoints(indexPairs(:, 1), :), width, height, dim);
% [currPointsConversion, currLLIndexes] = createPointsConversionTable(...
% 	currPoints(indexPairs(:, 2), :), width, height, dim);

% Select matched points.
projectedMatches1 = SURFPoints(prevPointsConversion(indexPairs(:, 1), :));
projectedMatches2 = SURFPoints(currPointsConversion(indexPairs(:, 2), :));

% DEBUG
% disp(projectedMatches1);

% Estimate the camera pose of current view relative to the previous view.
% The pose is computed up to scale, meaning that the distance between
% the cameras in the previous view and the current view is set to 1.
% This will be corrected by the bundle adjustment.
[relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(...
	projectedMatches1, projectedMatches2, cameraParams);

% prevCamMatrix = cameraMatrix(cameraParams, eye(3), [0, 0, 0]);
% currCamMatrix = cameraMatrix(cameraParams, relativeOrient, relativeLoc);
% prevWorldPoints = triangulate(projectedMatches1, projectedMatches2, prevCamMatrix, currCamMatrix);

% Add the current view to the view set.
vSet = addView(vSet, viewId, 'Points', SURFPoints(currPointsConversion));

% Store the point matches between the previous and the current views.
vSet = addConnection(vSet, 1, 2, 'Matches', indexPairs(inlierIdx,:));

% Get the table containing the previous camera pose.
prevPose = poses(vSet, 1);
prevOrientation = prevPose.Orientation{1};
prevLocation    = prevPose.Location{1};

% Compute the current camera pose in the global coordinate system
% relative to the first view.
orientation = relativeOrient * prevOrientation;
location    = prevLocation + relativeLoc * prevOrientation;
vSet = updateView(vSet, viewId, 'Orientation', orientation, ...
	'Location', location);

% updating variables before next iteration
prevPoints = currPoints;
prevPointsConversion = currPointsConversion;
prevFeatures = currFeatures;

%% Processing all the other images
for i = 3:numel(images)
    % Undistort the current image.
    I = images{i};

    % Detect, extract and match features.
    currPoints = detectSURFFeatures(I);
	
	[currPointsConversion, llIndexes] = createPointsConversionTable(...
		currPoints, width, height, dim);
	% Remove all points that does not fit in the front projection image
	currPoints = currPoints(llIndexes);
	% Remove all points that does not belong to the front projection
	frontIndexes = findFrontProjectionIndexes(currPointsConversion);
	if length(frontIndexes) < 8
		error(['Too less key points in front face for image ', num2str(i)]);
	end
	currPoints = currPoints(frontIndexes);
	for j = 1:length(currPointsConversion)
		currPointsConversion{j} = currPointsConversion{j}(frontIndexes, :);
	end
	
    currFeatures = extractFeatures(I, currPoints);
    indexPairs = matchFeatures(prevFeatures, currFeatures, ...
        'MaxRatio', .7, 'Unique',  true);

    % Select matched points.
    projectedMatches1 = SURFPoints(prevPointsConversion{1}(indexPairs(:, 1), :));
    projectedMatches2 = SURFPoints(currPointsConversion{1}(indexPairs(:, 2), :));

    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(...
        projectedMatches1, projectedMatches2, cameraParams);
	
    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', SURFPoints(currPointsConversion{1}));

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

    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);

    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);

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
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Refined Camera Poses');