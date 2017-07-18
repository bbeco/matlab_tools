function pointsConversion = createPointsConversionTable(points, width, height, dim)
	addpath(fullfile('coordinate_transform'));
	l = size(points, 1);
	% Each row is composed of 3 elements: a 1-by-2 LLimage point, the 1-by-2
	% perspective projection of that point and a logical value 1 if using back
	% hemisphere or 0 otherwise.
	pointsConversion = cell(1, 3);
	projectedCoord = zeros(l, 2);
	usingBackFace = zeros(l, 1, 'logical');
	u0 = dim/2;
	v0 = u0;
	f = 1;
	for i = 1:l
		[lat, long] = extractLLCoordinateFromImage(...
			points(i, 1), points(i, 2), width, height);
		[x, y, z] = LL2Cartesian(lat, long);
		m = perspectiveProjection([x, y, z], f, u0, v0);
		projectedCoord(i, :) = m;
		usingBackFace(i) = ~(z >= 0);
	end
	pointsConversion{1} = points;
	pointsConversion{2} = projectedCoord;
	pointsConversion{3} = usingBackFace;
end