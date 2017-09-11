function validFrame = selectFrame(...
	llPoints1, llPoints2, angularThreshold, width, height, quantileAnalysed)

	if ~isnumeric(llPoints1)
		llPoints1 = llPoints1.Location;
	end
	
	if ~isnumeric(llPoints2)
		llPoints2 = llPoints2.Location;
	end
	
	distances = 180/pi*computeAngularDistance(...
			llPoints1, llPoints2, width, height);
	
	if nargin < 6
		% Consider the 80th-percentile of the distances
		quantileAnalysed = 0.8;
	end
	
	% extract the 80th-percentile of distances
	q = quantile(distances, quantileAnalysed);
	% analyse only distances above q
	distances = distances(distances >= q);
	% take the median for robustness
	validFrame = median(distances) >= angularThreshold;
end

function angularDistance = computeAngularDistance(...
	imagePoint1, imagePoint2, width, height)
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
		angularDistance(i) = 2*asin(d/4);
	end
end