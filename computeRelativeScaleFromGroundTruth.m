function rs = computeRelativeScaleFromGroundTruth(...
		gtPoses, estimatedMotion, i, j)
% This function computes the right scale for motion vector between two
% views from the ground truth data. It takes the magnitude of the relative 
% motion from the ground truth data and divides it by the magnitude of the
% estimated motion.
%
%	Input:
%		-gtPoses: ground truth poses;
%		-estimatedMotion: the estimated distance
%		-i: the view ID for the last view;
%		-j: the view ID for the second-last view;
%
%	Output:
%		-rs: the relative scale
	if i <= j
		error(['Unable to compute relative scale between view ', ...
			num2str(i), ' and ', num2str(j)]);
	end
	
	rs = norm(gtPoses.Location{i} - gtPoses.Location{j})/norm(estimatedMotion);
end