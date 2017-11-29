function [absLocError, absOrientError] = computeAbsErrorNew(vSet, groundTruthPoses, frameUsed)
groundTruthPoses = groundTruthPoses(frameUsed, :);
groundTruthPoses = alignOrientation(groundTruthPoses);
groundTruthPoses = translateLocation(groundTruthPoses);
gtRelMotion = computeRelativeMotion(groundTruthPoses);

[vSet, ~] = normalizeCameraPosesWithGroundTruth(vSet, groundTruthPoses);
camPoses = poses(vSet);

estimatedRelMotion = computeRelativeMotion(camPoses);

absLocError = zeros(size(estimatedRelMotion, 1), 1);
% ZYX
absOrientError = zeros(size(estimatedRelMotion, 1), 3);
for i = 2:size(estimatedRelMotion, 1)
	absLocError(i) = ...
		norm(estimatedRelMotion.Location{i} - gtRelMotion.Location{i});
	estAngles = rotm2eul(estimatedRelMotion.Orientation{i}');
	gtAngles = rotm2eul(gtRelMotion.Orientation{i}');
	for j = 1:3
		absOrientError(i, j) = abs(estAngles(j) - gtAngles(j));
	end
end