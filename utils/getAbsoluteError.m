function errorTable = getAbsoluteError(location, orientation, ...
	groundTruthPoses)
	equalSize = size(location, 1) == size(orientation, 1) && ...
		size(orientation, 1) == size(groundTruthPoses, 1);
	if ~equalSize
		error('Unable to compare camera poses: estimation and GT size are different');
	end
	
	l = size(location, 1);
	
	orientationGT = zeros(l, 3);
	for i = 1:l
		orientationGT(i, :) = rotm2eul(groundTruthPoses.Orientation{i});
	end
	
	locationGT = cat(1, groundTruthPoses.Location{:});
	locError = abs(locationGT - location);
	orientError = abs(orientationGT - orientation)*180/pi;
	
	errorTable = table(locError(:, 1), locError(:, 2), locError(:, 3),...
		orientError(:, 3), orientError(:, 2), orientError(:, 1), ...
		'VariableNames', {'locErrorX', 'locErrorY', 'locErrorZ', ...
		'orientErrorX', 'orientErrorY', 'orientErrorZ'});
end