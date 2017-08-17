function [location, orientation] = computeRelativeMotion(poses)
	len = size(poses, 1);
	location = cell(len, 1);
	orientation = cell(len, 1);
	
	for i = len:-1:2
		location{i} = poses.Location{i} - poses.Location{i - 1};
		orientation{i} = poses.Orientation{i} * poses.Orientation{i - 1}';
	end
	orientation{1} = eye(3, 'like', poses.Orientation{1});
	location{1} = zeros(1, 3, 'like', poses.Location{1});
end