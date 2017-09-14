function angularDistance = computeAngularDistance(...
	imagePoint1, imagePoint2, width, height)
% This function is used by the frame filter to compute the angular distance
% between matching points in different views.

	len = size(imagePoint1, 1);
	angularDistance = zeros(len, 1);

	for i = 1:len
		[lat1, long1] = extractLLCoordinateFromImage(...
			imagePoint1(i, 1), imagePoint1(i, 2), width, height);
		[lat2, long2] = extractLLCoordinateFromImage(...
			imagePoint2(i, 1), imagePoint2(i, 2), width, height);

		x1 = LL2Cartesian(lat1, long1);
		x2 = LL2Cartesian(lat2, long2);

		d = norm(x1 - x2);
		% the following remaps d on the interval [-1, 1]. The result belongs to
		% [-pi/2, pi/2] so we have to add pi/2 for our purpose.
		angularDistance(i) = 2*asin(d/2);
	end
end