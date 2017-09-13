function [vSet, absoluteScale] = normalizeCameraPosesWithGroundTruth(...
		vSet, groundTruth)
% This function is an extract from the Matlab's visual odometry example found at
% https://it.mathworks.com/help/vision/examples/monocular-visual-odometry.html.

	camPoses = poses(vSet);
	locations = cat(1, camPoses.Location{:});
	magnitudes = sqrt(sum(locations.^2, 2));
	locationsGT = cat(1, groundTruth.Location{:});
	magnitudesGT = sqrt(sum(locationsGT.^2, 2));
	
	absoluteScale = median(magnitudesGT(2:end) ./ magnitudes(2:end));
	
	locations = locations .* absoluteScale;
	
	camPoses.Location = num2cell(locations, 2);
	
	vSet = updateView(vSet, camPoses);
end