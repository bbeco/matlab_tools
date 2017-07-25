function validIndexes = filterLLMatches(points1, points2, indexPairs, angularThreshold, width, height)
% This function filters the matches previously computed by matchFeatures(). It
% takes the array of index pairs previously returned by matchFeatures and 
% discards all the matches between points whose LL coordinates differs by, at 
% least, angularThreshold (in degrees).
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
	l = size(indexPairs, 1);
	validIndexes = zeros(l, 1, 'like', indexPairs);
	validIndexNumber = 0;
	conversionFactor = 180/pi;
	
	for i = 1:l
		[lat1, long1] = extractLLCoordinateFromImage(...
			points1(i, 1), points1(i,2), width, height);
		[lat2, long2] = extractLLCoordinateFromImage(...
			points2(i, 1), points2(i, 2), width, height);
		lat1 = conversionFactor*lat1;
		long1 = conversionFactor*long1;
		lat2 = conversionFactor*lat2;
		long2 = conversionFactor*long2;
		
		angularDistance = sqrt((lat1 - lat2)^2 + (long1 - long2)^2);
		if angularDistance > angularThreshold
			validIndexNumber = validIndexNumber + 1;
			validIndexes(validIndexNumber) = i;
		end
	end
	validIndexes = validIndexes(1:validIndexNumber);
end