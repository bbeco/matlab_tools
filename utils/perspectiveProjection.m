function m = perspectiveProjection(M, f, u0, v0)
%	This function compute the image coordinate of the points given as
%	input.
%
%       Input:
%           -M: an M-by-3 matrix of 3D world point to be transformed;
%           -f: the focal length in pixel
%           -u0: the u-coordinate of the principal point;
%           -v0: the v-coordinate of teh principal point.
%
%       Output:
%           -m: an M-by-2 matrix of 2D image points that are the projection
%               of the input points.
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

	P = [f, 0, u0, 0;
		0, f, v0, 0;
		0, 0, 1, 0];
	l = size(M, 1);
	m = zeros(l, 2);
	for i = 1:l
		x = M(i, :);
		x(4) = 1;
		image_point = P*x';
		m(i, :) = image_point(1:2)/image_point(3)';
	end
end
		
		
