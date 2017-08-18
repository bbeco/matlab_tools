function [pointsConversion, validIdx] = projectKeyPointDirections(...
	points, width, height, dim)
%
%	Input:
%		-
%
%	Output:
%		-pointsConversion: The data structure containing projections, face,
%		worldpoints and validity flag. Only the points that fit inside the image
%		plane are stored.
%		-pointIndexes: the indexes of image points that did not fall outside
%		image plane;
%
	
	if ~isnumeric(points)
		points = points.Location;
	end
	
	l = size(points, 1);

	pointsConversion = zeros(l, 2, 'like', points);
	validIdx = zeros(1, l, 'logical');
	realLength = 0;
	
	u0 = dim/2;
	v0 = u0;
	f = 1;
	for i = 1:l
		[lat, long] = extractLLCoordinateFromImage(...
			points(i, 1), points(i, 2), width, height);
		[x, y, z] = LL2Cartesian(lat, long);
		if z >= 0
			m = perspectiveProjection([x, y, z], f, u0, v0);
			if isequal(m > 0 & m <= dim, [1, 1])
				realLength = realLength + 1;
				validIdx(i) = true;
				pointsConversion(realLength, :) = m;
			end
		end
	end
	validIdx = validIdx(1:realLength);
	pointsConversion = pointsConversion(1:realLength, :);
end