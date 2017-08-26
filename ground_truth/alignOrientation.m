function poses = alignOrientation(poses)
	R = poses.Orientation{1};
	if isequal(R, eye(3))
		return;
	end
	
	for i = 2:size(poses, 1)
		poses.Orientation{i} = poses.Orientation{i} * R';
	end
	poses.Orientation{1} = eye(3, 'like', poses.Orientation{1});
end