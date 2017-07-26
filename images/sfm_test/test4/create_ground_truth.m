viewId = [1, 2, 3, 4, 5]';
Location = {
		[0, 0, 0;]
		[1.5000, 0.0056, 0.0000;]
		[3.0000, 0.8000, -0.0000;]
		[4.0000, 0.8000, -0.0000;]
		[4.1289, 4.2749, 0.0000];
	}
Orientation = {
		[0, 0, 0],
		[0, 20, 0],
		[0, 25, 0],
		[0, 50, 0],
		[0, 65, 0]
	};
groundTruthPoses = table(viewId, Location, Orientation);

save('images/sfm_test/test4/groundTruth.mat', 'groundTruthPoses');