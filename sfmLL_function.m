function [vSet, xyzPoints, reprojectionErrors, relLocation, ...
		relOrientation, pointsForEEstimationCounter, ...
		pointsForPoseEstimationCounter] = ...
		sfmLL_function(imageDir, ...
		computeRelativeScaleBeforeBundleAdjustment, maxAcceptedReprojectionError, ...
		filterMatches, angularThreshold, ...
		zMin, ...
		prefilterLLKeyPoints, maxLatitudeAngle, performBundleAdjustment, ...
		viewsWindowSize, groundTruthPoses)
	
	addpath(fullfile('ground_truth'));
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

	[height, width] = size(images{1});
	
	cameraParams = cameraParameters;
	
	vWindow = ViewWindow(viewsWindowSize);
	
	%compute relative motion between each pairs of views to evaluate the
	%error for each relative pose estimation
	groundTruthPoses = alignOrientation(groundTruthPoses);
	
	% Estimated locations and orientations for each pairs
	relLocation = cell(1, numel(images));
	relOrientation = cell(1, numel(images));
	
	% number of points used for E estimation and for pose estimation
	pointsForEEstimationCounter = zeros(numel(images), 1);
	pointsForPoseEstimationCounter = zeros(numel(images), 1);
	
		
	%% Processing first image
	% Load first image
	disp('Processing image 1');
	I = images{1};

	prevPoints = detectSURFFeatures(I);

	if prefilterLLKeyPoints
		indexes = filterLLPoints(prevPoints, maxLatitudeAngle, width, height);
		prevPoints = prevPoints(indexes, :);
	end

	[prevPointsConversion, prevLLIndexes, prevFrontIndex] = ...
		createPointsConversionTable(prevPoints, zMin, width, height);

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
	
	vSet = addView(vSet, viewId, 'Points', ...
		prevPointsConversion(prevFrontIndex, :),...
		'Orientation', eye(3, 'like', prevPoints.Location), 'Location', ...
		zeros(1, 3, 'like', prevPoints.Location));
	
	relLocation{1} = zeros(1, 3, 'double');
	relOrientation{1} = eye(3, 'double');
	
	addPoints(vWindow, viewId, prevPoints(prevFrontIndex, :), ...
		prevFeatures(prevFrontIndex, :), ...
		prevPointsConversion(prevFrontIndex, :));

	%% Processing all the other images
	for i = 2:numel(images)
		disp(['Processing image ', num2str(i)]);
		% Undistort the current image.
		I = images{i};

		% Detect, extract and match features.
		currPoints = detectSURFFeatures(I);

		if prefilterLLKeyPoints
			indexes = filterLLPoints(currPoints, maxLatitudeAngle, width, height);
			currPoints = currPoints(indexes, :);
		end

		[currPointsConversion, currLLIndexes, currFrontIndex] = ...
			createPointsConversionTable(currPoints, zMin, width, height);

		% Remove all points that does not fit in the front projection image
		currPoints = currPoints(currLLIndexes, :);
		if length(currPoints) < 8
			error(['Too less key points in front face for image ',...
				num2str(i)]);
		end

		currFeatures = extractFeatures(I, currPoints);
		indexPairs = matchFeatures(prevFeatures, currFeatures, ...
			'MaxRatio', .7, 'Unique',  true);

		if filterMatches
			validIndexes = filterLLMatches(prevPoints, currPoints, ...
				indexPairs, angularThreshold, width, height);
			indexPairs = indexPairs(validIndexes, :);
		end

		% Estimate the camera pose of current view relative to the previous view.
		% The pose is computed up to scale, meaning that the distance between
		% the cameras in the previous view and the current view is set to 1.
		% This will be corrected by the bundle adjustment.
		[relativeOrient, relativeLoc, ~, inliersIdx, iterations, ...
			indexPairs, pointsForEEstimationCounter(i), ...
			pointsForPoseEstimationCounter(i)] = ...
			helperEstimateRelativePose(...
			prevPointsConversion, currPointsConversion, ...
			prevFrontIndex, currFrontIndex, indexPairs, cameraParams);
		
		% skipping image if we expect the pose estimation to be wrong
		if pointsForEEstimationCounter(i)/pointsForPoseEstimationCounter(i) >...
				15
			warning(['Skipping image ', num2str(i)]);
			continue;
		end
		
		% Get the table containing the previous camera pose.
		prevPose = poses(vSet, i-1);
		prevOrientation = prevPose.Orientation{1};
		prevLocation    = prevPose.Location{1};
		
		% From the third image on, compute the relative scale
		if i >= 2 && computeRelativeScaleBeforeBundleAdjustment
% 			relativeScale = computeRelativeScale(vSet, i, cameraParams, maxAcceptedReprojectionError);
			relativeScale = computeRelativeScaleFromGroundTruth(...
				groundTruthPoses, i, i - 1);
			relativeLoc = relativeScale * relativeLoc;
		end
		
		relLocation{i} = relativeLoc;
		relOrientation{i} = relativeOrient;

% 		disp(['E estimated with ', num2str(iterations), ' interactions']);
		% Add the current view to the view set.
		vSet = addView(vSet, i, 'Points', ...
			currPointsConversion(currFrontIndex, :));
		
		addPoints(vWindow, i, currPoints(currFrontIndex, :), ...
			currFeatures(currFrontIndex, :), ...
			currPointsConversion(currFrontIndex, :));

		% Compute the current camera pose in the global coordinate system
		% relative to the first view.
		orientation = relativeOrient * prevOrientation;
		location    = prevLocation + relativeLoc * prevOrientation;
		vSet = updateView(vSet, i, 'Orientation', orientation, ...
			'Location', location);
		
		if i >= vWindow.WindowSize
			vSet = computeTrackAndCreateConnections(vSet, vWindow);
		end

		% Find point tracks across all views.
		tracks = findTracks(vSet);

		% Get the table containing camera poses for all views.
		camPoses = poses(vSet);

		% Triangulate initial locations for the 3-D world points.
		[xyzPoints, reprojectionErrors] = triangulateMultiview(tracks, ...
			camPoses, cameraParams);

		% Refine the 3-D world points and camera poses.
		if performBundleAdjustment
			[xyzPoints, camPoses, reprojectionErrors] = ...
				bundleAdjustment(xyzPoints, tracks, camPoses, ...
				cameraParams, 'FixedViewId', 1, 'PointsUndistorted', true);
		end

		% Store the refined camera poses.
		vSet = updateView(vSet, camPoses);

		prevFeatures = currFeatures;
		prevPoints = currPoints;
		prevPointsConversion = currPointsConversion;
		prevFrontIndex = currFrontIndex;
	end

