function [locError, orientError] = writeExperimentResultsOnFile(outputFile, vSet, groundTruth, ...
		pointsForEEstimation, pointsForPoseEstimation, tracksNumber, ...
		frameUsed)
	
	camPoses = poses(vSet);
	computeRelativeLocationError = false;
	
	% remove unused views from ground truth
	groundTruth = groundTruth(frameUsed, :);
	groundTruth.ViewId = camPoses.ViewId;
	
	[locError, orientError] = computePoseError(...
		camPoses.Location, camPoses.Orientation, ...
		groundTruth, computeRelativeLocationError, camPoses.ViewId);
	
	t = table(camPoses.ViewId, locError, ...
		orientError(:, 3), orientError(:, 2), orientError(:, 1), ...
		pointsForEEstimation(frameUsed), pointsForPoseEstimation(frameUsed), tracksNumber(frameUsed), ...
		'VariableNames', {'ViewId', 'LocationError', ...
		'XOrientationError', 'YOrientationError', 'ZOrientationError', ...
		'PointsForEEstimation', 'PointsForPoseEstimation', ...
		'tracksNumber'});

	len = height(t);
	
	% delete output file if it already exists
	if exist(outputFile, 'file')
		delete(outputFile);
	end
	
	writetable(t, outputFile, 'Range' , 'A3');

	t = table((1:size(frameUsed, 1))', frameUsed, ...
		'VariableNames', {'ImageNumber', 'Used'});
	writetable(t, outputFile, 'Range', ['A', num2str(3 + 3 + len)]);
end