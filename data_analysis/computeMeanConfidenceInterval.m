function ci = computeMeanConfidenceInterval(data)
% This function returns a row vector with the confidence interval for the sample
% mean of each data's column.

	if size(data, 1) == 1
		ci = zeros(1, size(data, 2));
		return;
	end
	
	% when data is a matrix, sampleStdDev is a row vector with the sample
	% standard deviation of each column.
	sampleStdDev = std(data);
	
	% This is the 97.5%-percentile of the Standard Normal distribution
	z = 1.96;
	ci = sampleStdDev/sqrt(size(data, 1))*z;
end