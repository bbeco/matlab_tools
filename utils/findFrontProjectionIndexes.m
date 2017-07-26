function frontIndex = findFrontProjectionIndexes(pointsConversionTable)
	l = size(pointsConversionTable{1}, 1);
	frontIndex = zeros(l);
	indexLength = 0;
	for i = 1:l
		if ~pointsConversionTable{2}(i)
			indexLength = indexLength + 1;
			frontIndex(indexLength) = i;
		end
	end
	frontIndex = frontIndex(1:indexLength);
end