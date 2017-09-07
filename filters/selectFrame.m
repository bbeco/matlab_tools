function [validFrame, goodMatches] = selectFrame(...
	llPoints1, llPoints2, angularThreshold, filteringMode)

	distances = 180/pi*computeAngularDistance(...
			llPoints1, llPoints2, height, width);
	if nargout > 1
		goodMatches = distances > angularThreshold;
	end
		
	if filteringMode == 1
		% Mode 1: Discard the corresponding points that are closer than
		% angularThreshold
		validFrame = true;
	elseif filteringMode == 2
		% Mode 2: Discard the whole frame if the average angular distance
		% between matches is below angularThreshold
		validFrame = mean(distances, 1) > angularThreshold;
	elseif filteringMode == 3
		% Mode 2: Discard the whole frame if the average angular distance
		% between matches is below angularThreshold
		validFrame = median(distances, 1) > angularThreshold;
	end
end

function angularDistance = computeAngularDistance(...
	imagePoint1, imagePoint2)
	len = size(imagePoint1, 1);
	angularDistance = zeros(len, 1);

	for i = 1:len
		[lat1, long1] = extractLLCoordinateFromImage(...
			llPoints1(i, :), height, width);
		[lat2, long2] = extractLLCoordinateFromImage(...
			llPoints2(i, :), height, width);

		x1 = ll2cartesian(lat1, long1);
		x2 = ll2cartesian(lat2, long2);

		d = norm(x1 - x2);
		% the following remap d on the interval [-1, 1]. The result belongs to
		% [-pi/2, pi/2] so we have to add pi/2 for our purpose.
		angularDistance(i) = 2*asin((d - 1)/2) + pi/2;
	end
end