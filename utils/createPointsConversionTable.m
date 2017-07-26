function [pointsConversion, pointIndexes] = createPointsConversionTable(points, width, height, dim)
% Questa funzione deve implementare la trasformazione delle coordinate cosi'
% come diceva corsini.
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
	addpath(fullfile('coordinate_transform'));
	l = size(points, 1);

	pointsConversion = zeros(l, 2, 'like', points.Location(1, 1));
	pointIndexes = zeros(l, 1);
	realLength = 0;
	
	u0 = dim/2;
	v0 = u0;
	f = 1;
	for i = 1:l
		[lat, long] = extractLLCoordinateFromImage(...
			points.Location(i, 1), points.Location(i, 2), width, height);
		[x, y, z] = LL2Cartesian(lat, long);
		if z >= 0
			m = perspectiveProjection([x, y, z], f, u0, v0);
			if isequal(m > 0 & m <= dim, [1, 1])
				realLength = realLength + 1;
				pointIndexes(realLength) = i;
				pointsConversion(realLength, :) = m;
			end
		end
	end
	pointIndexes = pointIndexes(1:realLength);
	pointsConversion = pointsConversion(1:realLength, :);
end