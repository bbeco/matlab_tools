function ci = computeMeanConfidenceInterval(data)
% This function returns a row vector with the confidence interval for the sample
% mean of each data's column.
	dataMean = mean(data);
	sampleStdDev = std(data);
	% This is the 97.5%-percentile of the Standard Normal distribution
	z = 1.96;
	ci = sampleStdDev/sqrt(size(dataMean, 2) - 1)*z;
end