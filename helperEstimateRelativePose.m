function [relOrientation, relLocation, validPtsFraction, inliersIndex,...
	i, indexPairs, pointsForEEstimationCounter, ...
	pointsForPoseEstimationCounter] = ...
	helperEstimateRelativePose(conversion1, conversion2, ...
	frontIdx1, frontIdx2, indexPairs, cameraParams)

	removeBackPtsBeforeEestimation = true;
	removeBackPointsBeforePoseEstimation = false;
	maxIterations = 1;

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
	pointsForEEstimationCounter = size(matchedPts1, 1);

	for i = 1:maxIterations
		% inliersIndex is a logical vector with the points that satisfy the
		% epipolar constraint.
		[E, inliersIndex, status] = estimateEssentialMatrix(matchedPts1, ...
			matchedPts2, cameraParams, 'MaxNumTrials', 50000, 'MaxDistance', 0.001);

		if status ~= 0
			error('Something is wrong with E estimation');
		end

% 		if sum(inliersIndex) / numel(inliersIndex) < .3
% 			warning('Inliers index low after E estimation');
% 			continue;
% 		end

% 		disp(['Inliers Index: ', num2str(sum(inliersIndex))]);
		if removeBackPointsBeforePoseEstimation
			% Remove inliers that belongs to the back hemisphere
			inliersIndex = inliersIndex(frontIdx1 & frontIdx2);
		end
% 		disp(['Frontal inliers index: ', num2str(sum(inliersIndex))]);

		% Valid pointsPtsIndex is the fraction of points that reproject in front of
		% the two cameras it should be above .8
		[relOrientation, relLocation, validPtsFraction] = relativeCameraPose(E, ...
			cameraParams, ...
			matchedPts1(inliersIndex, :), matchedPts2(inliersIndex, :));
		pointsForPoseEstimationCounter = ...
			size(matchedPts1(inliersIndex, :), 1);

		if validPtsFraction > .8
			return;
		end
	end
	
	warning('estimating camera pose: Maximum iterations limit reached');
end
