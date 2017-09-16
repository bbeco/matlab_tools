function plotMeanErrorPerView(...
		paramTable, sumLocError, sumXerror, sumYerror, sumZerror, frameUsed)
	%PLOTMEANERRORPERVIEW Creates plots wiht the error vs threshold
	%   Detailed explanation goes here
	w = unique(paramTable.WindowSize);
	rows = size(unique(paramTable.AngularThreshold), 1);
	meanLocError = zeros(rows, size(w, 1));
	meanOrientErrorX = zeros(size(meanLocError));
	meanOrientErrorY = zeros(size(meanLocError));
	meanOrientErrorZ = zeros(size(meanLocError));
	labels = cell(1, size(w, 1));
	for i = 1: size(w, 1)
		labels{i} = ['w=', num2str(w(i))];
		expIdx = paramTable.WindowSize ==  w(i);
		tmp = cat(1, frameUsed{1, :});
		meanLocError(:, i) = sumLocError(expIdx) ./ tmp(expIdx);
		meanOrientErrorX(:, i) = sumXerror(expIdx) ./ tmp(expIdx);
		meanOrientErrorY(:, i) = sumYerror(expIdx) ./ tmp(expIdx);
		meanOrientErrorZ(:, i) = sumZerror(expIdx) ./ tmp(expIdx);
	end
	x = paramTable.AngularThreshold(expIdx);
	figure
	plot(x, meanLocError, '-+');
	legend(labels);
	xlabel('angularThreshold');
	ylabel('MeanLocError');
	title('Mean location error');
	
	figure
	plot(x, meanOrientErrorX, '-*');
	legend(labels);
	xlabel('angularThreshold');
	ylabel('MeanOrientErrorX');
	title('Mean Orientation error X');
	
	figure
	plot(x, meanOrientErrorY, '-*');
	legend(labels);
	xlabel('angularThreshold');
	ylabel('MeanOrientErrorY');
	title('Mean Orientation error Y');
	
	figure
	plot(x, meanOrientErrorZ, '-*');
	legend(labels);
	xlabel('angularThreshold');
	ylabel('MeanOrientErrorZ');
	title('Mean Orientation error Z');
end

