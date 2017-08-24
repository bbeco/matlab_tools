function [locError, orientError] = ...
	computePoseError(estLocation, estOrientation, groundTruth, vIds)

	if nargin < 4
		vIds = groundTruth.ViewId;
	end
	
	if size(estLocation, 1) ~= size(estOrientation, 1) || ...
		size(estLocation, 1) ~= length(vIds)
		error('sizes of estimated locations and orientations are different');
	end
	
	len = size(estLocation, 1);
	lenGT = size(groundTruth, 1);
	
	locError = cell(len, 1);
	orientError = cell(len, 1);
	
	for i = 1:len
		for j = 1:lenGT
			if vIds(i) == groundTruth.ViewId(j)
				orientationGT = rotm2eul(groundTruth.Orientation{j}');
				locationGT = groundTruth.Location{j};
				
				orientError{i} = abs(rotm2eul(estOrientation{i}') - ...
					orientationGT);
				locError{i} = ...
					abs(estLocation{i} - locationGT)/norm(locationGT);
				break;
			end
		end
	end
end
	
	