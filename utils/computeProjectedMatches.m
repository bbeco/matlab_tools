function [projectedMatches1, projectedMatches2, usingBackFace] = computeProjectedMatches(matchedPoints1, matchedPoints2, dim, width, height)
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
	u0 = dim/2;
	v0 = u0;
	frontCount = 0;
	backCount = 0;
	addpath('coordinate_transform/');
	l = size(matchedPoints1, 1);
	frontEmisphereMatches1 = zeros(l, 2);
	frontEmisphereMatches2 = zeros(l, 2);
	backEmisphereMatches1 = zeros(l, 2);
	backEmisphereMatches2 = zeros(l, 2);

	% focal lenght for the new planar projection
	f = 1;

	for i = 1:size(matchedPoints1, 1)
		% 
		LLu1 = matchedPoints1.Location(i, 1);
		LLv1 = matchedPoints1.Location(i, 2);
		[lat, long] = extractLLCoordinateFromImage(LLu1, LLv1, width, height);
		[x1, y1, z1] = LL2Cartesian(lat, long);
		m1 = perspectiveProjection([x1, y1, z1], f, u0, v0);
		if ~isequal(m1 >= 0 & m1 <= dim, [1, 1])
			continue;
		end

		%
		LLu2 = matchedPoints2.Location(i, 1);
		LLv2 = matchedPoints2.Location(i, 2);
		[lat, long] = extractLLCoordinateFromImage(LLu2, LLv2, width, height);
		[x2, y2, z2] = LL2Cartesian(lat, long);
		m2 = perspectiveProjection([x2, y2, z2], f, u0, v0);
		if ~isequal(m2 >= 0 & m2 <= dim, [1, 1])
			continue;
		end

		if z1 >= 0 && z2 >= 0
			frontCount = frontCount + 1;
			frontEmisphereMatches1(frontCount, :) = m1;
			frontEmisphereMatches2(frontCount, :) = m2;
		elseif z1 < 0 && z2 < 0
			backCount = backCount + 1;
			backEmisphereMatches1(backCount, :) = m1;
			backEmisphereMatches2(backCount, :) = m2;
		end
	end

	%selecting the emisphere with more feature matches
	if frontCount >= backCount
		projectedMatches1 = SURFPoints(frontEmisphereMatches1(1:frontCount, :));
		projectedMatches2 = SURFPoints(frontEmisphereMatches2(1:frontCount, :));
		usingBackFace = false;
	else
		projectedMatches1 = SURFPoints(backEmisphereMatches1(1:backCount, :));
		projectedMatches2 = SURFPoints(backEmisphereMatches2(1:backCount, :));
		usingBackFace = true;
	end
end