function validIndex = filterLLPoints(points, maxLatitudeAngle, width, height)
% This function return the indexes of points whose latitude is between
% -maxLatitudeAngle and +maxLatitudeAngle. It can be used to remove the
% keypoints that are close to the poles in an equirectangulare image.
%
%	Input:
%		-points: An M-by-2 array of keypoint locations;
%		-maxLatitudeAngle: this is the threshold in degrees used to discard 
%		keypoints. If a point has a bigger latitude value than this threshold, 
%		it is discarded;
%		-width: the width of the image where keypoints came from;
%		-height: the height of the image where keypoints came from.
%
%	Output:
%		-validIndex: an N-by-1 array of indexes of the points whose latitude (in
%		degrees) resides in [-maxLatitudeAngle, +maxLatitudeAngle].
%
%	Copyright 2017 Andrea Beconcini
%
%	This program is free software: you can redistribute it and/or modify
%	it under the terms of the GNU Lesser General Public License as published by
%	the Free Software Foundation, either version 3 of the License, or
%	any later version.
% 
%	This program is distributed in the hope that it will be useful,
%	but WITHOUT ANY WARRANTY; without even the implied warranty of
%	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%	GNU Lesser General Public License for more details.
% 
%	You should have received a copy of the GNU Lesser General Public License
%	along with this program.  If not, see <http://www.gnu.org/licenses/>.
%
	l = size(points, 1);
	validIndex = zeros(l, 1, 'uint32');
	indexesNumber = 0;
	
	for i = 1:l
		[lat, ~] = extractLLCoordinateFromImage(points.Location(i, 1), ...
			points.Location(i, 2), width, height);
		if lat*180/pi > maxLatitudeAngle
			continue;
		end
		
		indexesNumber = indexesNumber + 1;
		validIndex(indexesNumber) = i;
	end
	validIndex = validIndes(1:indexesNumber);
end