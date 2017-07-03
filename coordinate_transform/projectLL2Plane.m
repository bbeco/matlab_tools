function [u, v, z] = projectLL2Plane(lat, long, size)
%   compute the coordinate [u, v] of the projection of the 3D point whose LL
%	coordinates are given as input. This point coordinates are first transformed
%	in cartesian and then the z component is dropped.
%   The u, v plane is the same as the x, y plane and the u and v axes have the
%   same direction of the x and y axes respectively. The origin of the uv
%   coordinate system is put on the top left corner of the image.
% 
% 	Input:
% 		-lat: latitude value (in radians)
% 		-long: longitude value (in radians)
% 		
% 	Output:
% 		-[u, v, z]: an 1x3 vector with the resulting projected coordinates. The
% 		z-coordinate can be useful to distinguish the emisphere the original 3D
% 		point belongs to.
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
	[x, y, z] = LL2Cartesian(lat, long);
	% rescaling projection coordinate accordingly to size.
	% This is because u v coordinates are assigned to a SURFPoints.Location
	% which expects positive coordinates.
	u = (x + 1)/2*size;
	v = (y + 1)/2*size;
end