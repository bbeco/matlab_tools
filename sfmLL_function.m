function [vSet, xyzPoints, reprojectionErrors] = ...
		sfmLL_function(imageDir, ...
		computeRelativeScaleBeforeBundleAdjustment, maxAcceptedReprojectionError, ...
		filterMatches, angularThreshold, ...
		projectExtractedKeyPointDirections, dim, f, ...
		prefilterLLKeyPoints, maxLatitudeAngle, performBundleAdjustment, ...
		viewWindowSize)
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

	vWindow = ViewWindow(viewWindowSize);
	
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
	
	addPoints(vWindow, 1, prevPoints, prevFeatures, prevPointsConversion);

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
		
		addPoints(vWindow, i, currPoints, currFeatures, currPointsConversion);
		computeTrackAndCreateConnections(vSet, vWindow, indexPairs);

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

