function rs = computeRelativeScaleFromGroundTruth(gtPoses, i, j)
% This function computes the right scale for motion vector between two
% views from the ground truth data.
%
%	Input:
%		-gtPoses: ground truth poses;
%		-i: the view ID for the last view;
%		-j: the view ID for the second-last view;
%
%	Output:
%		-rs: the relative scale
	if i <= j
		error(['Unable to compute relative scale between view ', ...
			num2str(i), ' and ', num2str(j)]);
	end
	
	rs = norm(gtPoses.Location{i} - gtPoses.Location{j})/...
		norm(gtPoses.Location{2});
end