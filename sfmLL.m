clear all;
addpath('utils/');
%imageDir = fullfile('images', 'essential_matrix_test', {'ll0.png', 'll1.png', 'll2.png', 'll3.png', 'll4.png', 'll5.png', 'll6.png'});
imageDir = fullfile('images', 'essential_matrix_test', {'ll0.png', 'll6.png'});
	
imds = imageDatastore(imageDir);

% Display the images.
figure
montage(imds.Files, 'Size', [4, 2]);

% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
    I = readimage(imds, i);
    images{i} = rgb2gray(I);
end

title('Input Image Sequence');

% Load first image
I = images{1};

prevPoints = detectSURFFeatures(I);

% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(I, prevPoints);

% Create an empty viewSet object to manage the data associated with each
% view.
vSet = viewSet;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;

%projecting points
[height, width] = size(I);
dim = min(height, width);
f = 1;
u0 = dim/2;
v0 = u0;
K = [
	f,	0,	u0;
	0,	f,	v0;
	0,	0,	1;];
cameraParams = cameraParameters('IntrinsicMatrix', K');

prevPointsConversion = createPointsConversionTable(...
	prevPoints.Location, width, height, dim);

vSet = addView(vSet, viewId, 'Points', prevPointsConversion{2},'Orientation', ...
    eye(3, 'like', prevPoints.Location), 'Location', ...
    zeros(1, 3, 'like', prevPoints.Location));

for i = 2:numel(images)
    % Undistort the current image.
    I = images{i};

    % Detect, extract and match features.
    currPoints = detectSURFFeatures(I);
	currPointsConversion = createPointsConversionTable(...
		currPoints.Location, width, height, dim);
    currFeatures = extractFeatures(I, currPoints);
    indexPairs = matchFeatures(prevFeatures, currFeatures, ...
        'MaxRatio', .7, 'Unique',  true);
	
	% The pairs to be used for motion estimation must be refined in order to 
	% consider the additional projection required by full spherical cameras.
	[refinedIndexPairs, usingBackFace] = refineIndexPairs(...
		indexPairs, prevPoints.Location, currPoints.Location, width, height);

    % Select matched points.
    projectedMatches1 = prevPointsConversion{2}(refinedIndexPairs(:, 1), :);
    projectedMatches2 = currPointsConversion{2}(refinedIndexPairs(:, 2), :);

    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(...
        projectedMatches1, projectedMatches2, cameraParams);
	
	% Fix axes direction when using backFace projection.
	if usingBackFace
		relativeLoc(1) = -relativeLoc(1);
		relativeLoc(3) = -relativeLoc(3);
	end
	
    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', currPointsConversion{2});

    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', refinedIndexPairs(inlierIdx,:));

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
    prevPoints   = currPoints;
	prevPointsConversion = currPointsConversion;
end

% Display camera poses.
camPoses = poses(vSet);
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

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
xlabel('X');
ylabel('Y');
zlabel('Z');

title('Refined Camera Poses');