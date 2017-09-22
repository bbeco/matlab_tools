function rImg = rotateLL(img, rot)
%	This function perform a rotation of the LL encoded image img accordingly to 
%	the rotation matrix provided.
%
% 	The algorithm works as follow: for each point in the rotated image
% 		1) projects it to on the unit sphere and compute the x, y, z coordinates
% 		2) perform the rotations around the X, Y and Z-axes (in this order).
% 			NB: it performs a rotation in the OPPOSITE direction because we 
% 			are moving from the rotated image to the original one in order to 
% 			sample pixel (see backward warping).
% 		3) convert the point back in LL coordinates
% 		4) sample the corresponding pixel from the original image.
%	
%	Input:
%		-img: The original image in LL format
%		-x: The value for the rotation around the x-axe
%		-y: The value for the rotation around the y-axe
%		-z: The value for the roation around the z-axe
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
			[rlat, rlong] = extractLLCoordinateFromImage(j, i, width, height);
			
            p = LL2Cartesian(rlat,rlong);
			p = rot * p;
            [lat, long] = cartesian2LL(p);
			
            %mapping
            u = (long + pi)/(2*pi)*width;
            v = (pi/2 - lat)/pi*height;
			
			%sampling
			%TODO use interpolation when sampling (interp?)
            rImg(i, j) = img(max(1, ceil(v)), max(1, ceil(u)));
        end
    end
end
