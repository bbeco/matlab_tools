function rs = computeRelativeScaleFromGroundTruth(gtPoses, i, j)
% This function computes the right scale for motion vector between two
% views from the ground truth data. In this way the relative scale is
% exactly the absolute scale of the scene.
%
%	Input:
%		-gtPoses: ground truth poses;
%		-i: the view ID for the last view;
%		-j: the view ID for the second-last view;
%
%	Output:
%		-rs: the relative scale
	rs = norm(gtPoses.Location{j} - gtPoses.Location{i});
end