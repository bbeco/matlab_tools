function myCameraPlot(vSet, groundTruth, vIds)
	if nargin < 3
		vIds = 1:vSet.NumViews;
	end
	camPoses = poses(vSet, vIds);
	figure;
	plotCamera(camPoses, 'Size', 0.2);
	hold on
	plotCamera(groundTruth(vIds, :), 'Size', 0.2, 'Color', [0 1 0]);
	xlabel('X');
	ylabel('Y');
	zlabel('Z');

	% Display the 3-D points.
	pcshow([0 0 0], 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
		'MarkerSize', 45);
	grid on
	hold off
end