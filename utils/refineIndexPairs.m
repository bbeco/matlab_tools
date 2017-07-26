function [refinedIndexPairs, usingBackFace] = refineIndexPairs(indexPairs, points1, points2, width, height)
	dim = min(height, width);
	% Optical center coordinates on the image plane
	u0= dim/2;
	v0 = u0;

	backFaceIndexPairs = zeros(size(indexPairs), 'like', indexPairs);
	frontFaceIndexPairs = zeros(size(indexPairs), 'like', indexPairs);
	
	frontCount = 0;
	backCount = 0;
	addpath('coordinate_transform/');
	l = size(indexPairs, 1);

	% focal lenght for the new planar projection
	f = 1;

	for i = 1:l
		p1 = points1(indexPairs(i, 1), :);
		p2 = points2(indexPairs(i, 2), :);
		
		% 
		[lat, long] = extractLLCoordinateFromImage(p1(1), p1(2), width, height);
		[x1, y1, z1] = LL2Cartesian(lat, long);
		m1 = perspectiveProjection([x1, y1, z1], f, u0, v0);
		if ~isequal(m1 >= 0 & m1 <= dim, [1, 1])
			continue;
		end

		%
		[lat, long] = extractLLCoordinateFromImage(p2(1), p2(2), width, height);
		[x2, y2, z2] = LL2Cartesian(lat, long);
		m2 = perspectiveProjection([x2, y2, z2], f, u0, v0);
		if ~isequal(m2 >= 0 & m2 <= dim, [1, 1])
			continue;
		end

		if z1 >= 0 && z2 >= 0
			frontCount = frontCount + 1;
			frontFaceIndexPairs(frontCount, :) = indexPairs(i, :);
		elseif z1 < 0 && z2 < 0
			backCount = backCount + 1;
			backFaceIndexPairs(backCount, :) = indexPairs(i, :);
		end
	end

	%selecting the emisphere with more feature matches
	if true
		refinedIndexPairs = frontFaceIndexPairs(1:frontCount, :);
		usingBackFace = false;
	else
		refinedIndexPairs = backFaceIndexPairs(1:backCount, :);
		usingBackFace = true;
	end
end