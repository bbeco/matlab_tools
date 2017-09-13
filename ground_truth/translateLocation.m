function poses = translateLocation(poses)
	t = poses.Location{1};
	
	if isequal(t, [0 0 0])
		return;
	end
	
	for i = 1:size(poses, 1)
		poses.Location{i} = poses.Location{i} - t;
	end
end