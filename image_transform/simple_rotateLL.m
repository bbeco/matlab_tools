function rImg = rotateLL(img, rot)
%	This is an old function I used for testing porpuses, it should not be used.
%
%	Output:
%		rImg: The rotated LL image
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

    [height, width, c] = size(img);
	if c > 1
		error('img is expected to be a single channel gray scale image');
	end
	
    rImg = zeros(height, width, 'like', img);
	
    for i = 1:height
        for j = 1:width
			%unmapping
			[lat, long] = extractLLCoordinateFromImage(j, i, width, height);
			
            p = LL2Cartesian(lat,long);
			p = rot' * p;
			%rotate 90degree around z
			tmp = p(1);
			p(1) = p(2);
			p(2) = -tmp;
			
            [rlat, rlong] = cartesian2LL(p);
			
            %mapping
            u = (rlong + pi)/(2*pi)*width;
            v = (pi/2 - rlat)/pi*height;
			
			%sampling
% 			TODO use interpolation when sampling (interp?)
            rImg(max(1, floor(v)), max(1, floor(u))) = img(i, j);
        end
    end
end
