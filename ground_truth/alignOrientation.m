function poses = alignOrientation(poses)
	R = poses.Orientation{1}';
	if isequal(poses.Orientation{1}, eye(3))
		return;
	end
	
	for i = 1:size(poses, 1)
		poses.Orientation{i} = poses.Orientation{i} * R;
		poses.Location{i} = poses.Location{i} * R;
	end
end