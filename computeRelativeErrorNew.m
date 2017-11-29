function [relLocError, relOrientError] = computeRelativeErrorNew(vSet, groundTruthPoses, frameUsed)
groundTruthPoses = groundTruthPoses(frameUsed, :);
groundTruthPoses = alignOrientation(groundTruthPoses);
groundTruthPoses = translateLocation(groundTruthPoses);
gtRelMotion = computeRelativeMotion(groundTruthPoses);

[vSet, ~] = normalizeCameraPosesWithGroundTruth(vSet, groundTruthPoses);
camPoses = poses(vSet);

estimatedRelMotion = computeRelativeMotion(camPoses);

relLocError = zeros(size(estimatedRelMotion, 1), 1);
% ZYX
relOrientError = zeros(size(estimatedRelMotion, 1), 3);
for i = 2:size(estimatedRelMotion, 1)
	relLocError(i) = ...
		norm(estimatedRelMotion.Location{i} - gtRelMotion.Location{i})/norm(gtRelMotion.Location{i});
	estAngles = rotm2eul(estimatedRelMotion.Orientation{i}');
	gtAngles = rotm2eul(gtRelMotion.Orientation{i}');
	for j = 1:3
		relOrientError(i, j) = abs(estAngles(j) - gtAngles(j))/abs(gtAngles(j));
	end
end