function [vSet, xyzPoints, reprojectionErrors, ...
		pointsForEEstimationCounter, ...
		pointsForPoseEstimationCounter, tracksSize, frameUsed] = ...
		sfmLL_function(imageDir, ...
		computeRelativeScaleBeforeBundleAdjustment, maxAcceptedReprojectionError, ...
		filterMatches, angularThreshold, ...
		zMin, ...
		prefilterLLKeyPoints, maxLatitudeAngle, ...
		performGlobalBundleAdjustment, performWindowedBundleAdjustment, ...
		viewsWindowSize, firstFrame, lastFrame, ...
		baAbsoluteTolerance, baRelativeTolerance, baMaxIterations, ...
		seqFilterAngularThreshold, seqFilterQuantile)
	
	addpath(fullfile('ground_truth'));
	imds = imageDatastore(imageDir);

	% Suppress figure warnings
	warning('off', 'images:initSize:adjustingMag');

	% Display the images.
% 	figure
% 	montage(imds.Files, 'Size', [4, 2]);

	% Convert the images to grayscale.
	imgNumber = lastFrame - firstFrame + 1;
	images = cell(1, lastFrame);
	parfor i = 1:lastFrame
		I = readimage(imds, i);
		images{i} = rgb2gray(I);
	end

	% title('Input Image Sequence');

	[height, width] = size(images{1});
	
	cameraParams = cameraParameters;
	
	vWindow = ViewWindow(viewsWindowSize);
	
	% number of points used for E estimation and for pose estimation
	pointsForEEstimationCounter = zeros(numel(images), 1);
	pointsForPoseEstimationCounter = zeros(numel(images), 1);
	
	% This stores the number of pointTracks found for every windowed bundle
	% adjustment. This vector is oversized because it is simpler to update it
	tracksSize = zeros(1, imgNumber);
	
	% This stores the average of the 3D points reprojection error every time we
	% perform either a windowed bundle adjustment or a global bundle adjustment.
	% This vector needs to be big enough to store a single value for each
	% windowed adjustment + 1 (because of the final global adjustment). We
	% allocate as many values as the number of images to be processed though,
	% because it easier to deal with its index.
	reprojectionErrors = zeros(1, imgNumber + 1);
	
	% The following saves which image has been used for pose estimation and
	% which not.
	frameUsed = zeros(1, imgNumber, 'logical');
	
	% The following support view set is used to store every possible connection
	% and track in order to estimate the relative scale for each translation
	% vector.
	supportViewSet = viewSet;
	
	%% Processing first image
	% Load first image
	disp('Processing image 1');
	I = images{firstFrame};
	frameUsed(firstFrame) = true;

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
	prevId = 1;
	
	vSet = addView(vSet, viewId, 'Points', ...
		prevPointsConversion,...
		'Orientation', eye(3, 'like', prevPoints.Location), 'Location', ...
		zeros(1, 3, 'like', prevPoints.Location));
	
	supportViewSet = addView(supportViewSet, viewId, 'Points', ...
		prevPointsConversion, ...
		'Orientation', eye(3, 'like', prevPoints.Location), ...
		'Location', zeros(1, 3, 'like', prevPoints.Location));
	
	addPoints(vWindow, viewId, prevPoints, ...
		prevFeatures, ...
		prevPointsConversion);

	%% Processing all the other images
	for i = (firstFrame + 1):lastFrame
		disp(['Processing image ', num2str(i)]);
		% Undistort the current image.
		I = images{i};
		currId = i;

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
			warning(['Too less key points in front face for image ',...
				num2str(i)]);
			warning(['Skipping image ', num2str(i)]);
			frameUsed(i) = false;
			continue;
		end

		currFeatures = extractFeatures(I, currPoints);
		indexPairs = matchFeatures(prevFeatures, currFeatures, ...
			'MaxRatio', .6, 'Unique',  true);

		if filterMatches
			validIndexes = filterLLMatches(prevPoints, currPoints, ...
				indexPairs, angularThreshold, width, height);
			indexPairs = indexPairs(validIndexes, :);
		end
		
		if sum(currFrontIndex(indexPairs(:, 2))) < 10
			warning(['Skipping frame: ', num2str(i)]);
			frameUsed(i) = false;
			continue;
		end
		
		% Display correspondences
% 		figure;
% 		showMatchedFeatures(images{i-1}, images{i}, ...
% 			prevPoints(indexPairs(:, 1), :), ...
% 			currPoints(indexPairs(:, 2), :), 'montage');
		
		% select frame in a sequence
		if ~selectFrame(prevPoints(indexPairs(:, 1), :), ...
				currPoints(indexPairs(:, 2), :), seqFilterAngularThreshold, ...
				width, height, seqFilterQuantile)
			% skip this frame
			frameUsed(i) = false;
			warning(['Skipping image: ', num2str(i)]);
			continue;
		end
		
		%from now on, the frame can no longer be discarded
		frameUsed(i) = true;
		
		viewId = viewId + 1;

		% Estimate the camera pose of current view relative to the previous view.
		% The pose is computed up to scale, meaning that the distance between
		% the cameras in the previous view and the current view is set to 1.
		% This will be corrected by the bundle adjustment.
		[relativeOrient, relativeLoc, validPtsFraction, inliersIdx, iterations, ...
			indexPairs, pointsForEEstimationCounter(i), ...
			pointsForPoseEstimationCounter(i)] = ...
			helperEstimateRelativePose(...
			prevPointsConversion, currPointsConversion, ...
			prevFrontIndex, currFrontIndex, indexPairs, cameraParams, ...
			50000, 100);
		
% 		% show matches
% 		showMatches(images{prevId}, images{i}, ...
% 				prevPoints(indexPairs(:, 1), :), ...
% 				currPoints(indexPairs(:, 2), :), [-1, -1]);
% 		while true
% 			[x, y] = ginput(1);
% 			display([x, y]);
% 			if isempty([x, y])
% 				break;
% 			end
% 			showMatches(images{prevId}, images{i}, ...
% 				prevPoints(indexPairs(:, 1), :), ...
% 				currPoints(indexPairs(:, 2), :), [x, y]);
% 		end
		
		% Get the table containing the previous camera pose.
		prevPose = poses(vSet, viewId - 1);
		prevOrientation = prevPose.Orientation{1};
		prevLocation    = prevPose.Location{1};
		
		% Compute the current camera pose in the global coordinate system
		% relative to the first view.
		orientation = relativeOrient * prevOrientation;
		location    = prevLocation + relativeLoc * prevOrientation;
		
		supportViewSet = addView(supportViewSet, viewId, ...
			'Points', currPointsConversion, ...
			'Orientation', orientation, ...
			'Location', location);
		
		% store the matches between the current image and the previously
		% analysed one
		supportViewSet = addConnection(supportViewSet, viewId - 1, viewId, ...
			'Matches', indexPairs);
		
		% From the third image on, compute the relative scale
		if viewId > 2 && computeRelativeScaleBeforeBundleAdjustment
			relativeScale = computeRelativeScale(...
				supportViewSet, viewId, cameraParams, ...
				maxAcceptedReprojectionError);
% 			relativeScale = computeRelativeScaleFromGroundTruth(...
% 				groundTruthPoses, relativeLoc, currId, prevId);
			location = prevLocation + ...
				relativeScale * relativeLoc * prevOrientation;
			supportViewSet = updateView(supportViewSet, viewId, ...
			'Location', location);
		end
		
		% Add the current view to the view set.
		vSet = addView(vSet, viewId, ...
			'Points', currPointsConversion, ...
			'Orientation', orientation, ...
			'Location', location);
		
		addPoints(vWindow, viewId, currPoints, ...
			currFeatures, ...
			currPointsConversion);
		
		addConnection(vWindow, viewId - 1, viewId, indexPairs);
		
		if viewId >= vWindow.WindowSize
			vSet = computeTrackAndCreateConnections(vSet, vWindow);

			if performWindowedBundleAdjustment
				% Find point tracks across all views.
				tracks = findTracks(vSet, vWindow.Views.ViewId);
				
				tracksSize(i) = length(tracks);

				% Get the table containing camera poses for all views.
				camPoses = poses(vSet, vWindow.Views.ViewId);

				% Triangulate initial locations for the 3-D world points.
				[xyzPoints, ~] = triangulateMultiview(tracks, ...
					camPoses, cameraParams);

				% Fixed views. This is to keep the scale fixed
				fixedViews = vWindow.Views.ViewId([1, 2])';
				
				% Refine the 3-D world points and camera poses.
				[xyzPoints, camPoses, tmpReprojectionErrors] = ...
					bundleAdjustment(xyzPoints, tracks, camPoses, ...
					cameraParams, 'FixedViewIDs', fixedViews, ...
					'PointsUndistorted', true, ...
					'AbsoluteTolerance', baAbsoluteTolerance, ...
					'MaxIterations', baMaxIterations, ...
					'RelativeTolerance', baRelativeTolerance, 'Verbose', true);

				reprojectionErrors(i) = mean(tmpReprojectionErrors);
				% Store the refined camera poses.
				vSet = updateView(vSet, camPoses);
				supportViewSet = updateView(supportViewSet, camPoses);
			end
		end

		prevId = currId;
		prevFeatures = currFeatures;
		prevPoints = currPoints;
		prevPointsConversion = currPointsConversion;
		prevFrontIndex = currFrontIndex;
	end
	
	% Find point tracks across all views.
	tracks = findTracks(vSet);

	% Get the table containing camera poses for all views.
	camPoses = poses(vSet);

	% Triangulate initial locations for the 3-D world points.
	[xyzPoints, ~] = triangulateMultiview(tracks, ...
		camPoses, cameraParams);
	
	% Refine the 3-D world points and camera poses.
	if performGlobalBundleAdjustment
		[xyzPoints, camPoses, tmpReprojectionErrors] = ...
			bundleAdjustment(xyzPoints, tracks, camPoses, ...
			cameraParams, 'FixedViewIDs', [1, 2], 'PointsUndistorted', true, ...
			'Verbose', true, ...
			'AbsoluteTolerance', baAbsoluteTolerance, ...
			'MaxIterations', baMaxIterations, ...
			'RelativeTolerance', baRelativeTolerance);
		reprojectionErrors(i + 1) = median(tmpReprojectionErrors);

		% Store the refined camera poses.
		vSet = updateView(vSet, camPoses);
	end
end
