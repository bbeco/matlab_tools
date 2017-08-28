function [pointsConversion, validPoints, frontalPointIndex] = ...
		createPointsConversionTable(points, zMin, width, height)
% This is an alternative way to translate the 3D directions of spherical images.
% This function divides every 3D points by its z-component so that every key 
% point is suitable to compute the essential matrix.
% To avoid numerical error, all the points whose absolute z-component is below 
% the threshold zMin are discarded. validPoints contains the indexes of the 
% points that have been successfully divided by their third component.
%	Input:
%		-points: The set of LL key points to be translated;
%		-zMin: The threshold value for the third components of the points. All 
%			the key points whose 3rd component modulus is below these threshold 
%			are discarded;
%		-width: the width of the LL image where the points came from. Used to 
%			extract latitude and longitude value;
%		-heigth: the height of the LL image where the points came from.
%
%	Output:
%		-pointsConversion: an array that contains the translated points
%		-validPoints: a logical array with the points that have been 
%			successfully translated;
%		-frontalPointIndex: a logical array with the points that belongs to the 
%			frontal hemisphere of the spherical images.
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
	conversionLength = 0;
	pointsConversion = zeros(l, 2);
	% all invalid at the beginning
	validPoints = zeros(1, l, 'logical');
	frontalPointIndex = zeros(1, l, 'logical');
	
	if ~isnumeric(points)
		points = points.Location;
	end
	
	for i = 1:l
		[lat, long] = extractLLCoordinateFromImage(points(i, 1), ...
			points(i, 2), width, height);
		[x, y, z] = LL2Cartesian(lat, long);
		
		if abs(z) < zMin
			continue;
		end
		
		conversionLength = conversionLength + 1;
		pointsConversion(conversionLength, :) = [x/z y/z];
		validPoints(i) = true;
		if z > 0
			frontalPointIndex(conversionLength) = true;
		end
	end
	pointsConversion = pointsConversion(1:conversionLength, :);
	frontalPointIndex = frontalPointIndex(1:conversionLength);
end
