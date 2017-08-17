function errorTable = getRelativeError(location, orientation, ...
	groundTruthPoses)

	equalSize = size(location, 1) == size(orientation, 1) && ...
		size(orientation, 1) == size(groundTruthPoses, 1);
	if ~equalSize
		error('Unable to compare camera poses: estimation and GT size are different');
	end
	
	for i = size(location, 1):-1:2
		location(i, :) = location(i, :) - location(i - 1, :);
		orientation{i} = orientation{i} * orientation{i - 1}';
		groundTruthPoses.Location{i} = groundTruthPoses.Location{i} - groundTruthPoses.Location{i - 1};
		groundTruthPoses.Orientation{i} = groundTruthPoses.Orientation{i} * groundTruthPoses.Orientation{i - 1}';
	end
	
	errorTable = getAbsoluteError(location, orientation, ...
		locationGT, orientatinoGT);
end