function translatedOrient = orientations2euler(orientation)
	len = size(orientation, 1);
	translatedOrient = cell(len, 1);
	for i = 1:len
		translatedOrient{i} = rotm2eul(orientation{i}');
	end
end