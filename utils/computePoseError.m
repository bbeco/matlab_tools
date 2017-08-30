function [locError, orientError, relLocError, relOrientError] = ...
	computePoseError(estLocation, estOrientation, groundTruth, vIds)

	if nargin < 4
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
	
	locError = cell(len, 1);
	orientError = cell(len, 1);
	relLocError = cell(len, 1);
	relOrientError = cell(len, 1);
	
	% If the GT's first camera orientation is different from eye(3), re-align
	% every GT's camera orientation.
	groundTruth = alignOrientation(groundTruth);
	
	relPosesGT = computeRelativeMotion(groundTruth);
	
	relPoses = computeRelativeMotion(...
			table((1:len)', estLocation, estOrientation,...
			'VariableNames', {'ViewId', 'Location', 'Orientation'}));
	
	for i = 1:len
		currLocGT = groundTruth.Location{i};
		currOrientGT = groundTruth.Orientation{i};

		orientError{i} = abs(...
			rotm2eul(currOrientGT') - rotm2eul(estOrientation{i}'));
		locError{i} = abs(currLocGT - estLocation{i});
		
		relLocGT = relPosesGT.Location{i};
		relOrientGT = relPosesGT.Orientation{i};

		relLocError{i} = relLocGT - relPoses.Location{i};
		relOrientError{i} = rotm2eul(relOrientGT') - ...
			rotm2eul(relPoses.Orientation{i}');
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
	
	