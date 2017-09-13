function [locError, orientError] = ...
	computePoseError(estLocation, estOrientation, groundTruth, ...
	computeRelativeErrors, vIds)
% This function compute the absolute value of the error for both location and
% orientation. The location error is the euclidean distance between the
% estimated pose and the relative ground truth while the orientation error is
% returned as the angular error around the three axis XYZ.

	if nargin < 5
		vIds = groundTruth.ViewId;
	else
		len = length(vIds);
		indexes = zeros(1, len);
		for j = 1:len
			[~, ~, indexes(j)] = findPose(groundTruth, vIds(j));
		end
		groundTruth = groundTruth(indexes, :);
	end
	
	if size(estLocation, 1) ~= size(estOrientation, 1) || ...
		size(estLocation, 1) ~= length(vIds)
		error('sizes of estimated locations and orientations are different');
	end
	
	len = size(estLocation, 1);
	
	locError = zeros(len, 1);
	orientError = zeros(len, 3);
	
	for i = 1:len
		currLocGT = groundTruth.Location{i};
		currOrientGT = rotm2eul(groundTruth.Orientation{i}');

		orientError(i, :) = abs(...
			currOrientGT - rotm2eul(estOrientation{i}'));
		locError(i) = norm(currLocGT - estLocation{i});
		
		if computeRelativeErrors
			locError(i) = locError(i)/norm(currLocGT);
		end
	end
end

function [location, orientation, index] = findPose(poses, vId)
	for i = 1:size(poses, 1)
		if poses.ViewId(i) == vId
			index = i;
			location = poses.Location{i};
			orientation = poses.Orientation{i};
			return;
		end
	end
	error('Unable to find a camera pose with the given id');
end
	
	