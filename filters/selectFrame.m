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