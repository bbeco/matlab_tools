function [dm_maxDisparity, dm_horDisparity] = computeMaxDisparity(gray1, gray2, qy, qx)
%COMPUTEMAXDISPARITY detects and matches features in the given images and
	%returns the maximum disparity value.
	
	if ~exist('qy', 'var')
	% This is the quantile analysed for disparities
		qy = 0.9;
	end
	if ~exist('qx', 'var')
		qx = 0.8;
	end
	
	points1 = detectSURFFeatures(gray1);
	points2 = detectSURFFeatures(gray2);

	features1 = extractFeatures(gray1, points1, 'Upright', true);
	features2 = extractFeatures(gray2, points2, 'Upright', true);

	indexPairs = matchFeatures(features1, features2);

	distances = abs(points1.Location(indexPairs(:, 1), :) - ...
		points2.Location(indexPairs(:, 2), :));

	dm_maxDisparity = ceil(quantile(sort(distances(:, 2)), qy));
	dm_horDisparity = ceil(quantile(sort(distances(:, 1)), qx));
end

