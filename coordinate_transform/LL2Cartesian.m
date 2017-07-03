function [x, y, z] = LL2Cartesian(lat, long)
%   compute the cartesian coordinates of the given point on the unitary 
%   sphere. The coordinate system has axes that satisfy the right-hand 
%   rule; the x-axis points right, the y-axis points down and the z-axis 
%	points forward.
% 
% 	Input:
% 		-lat: latitude value (in radians)
% 		-long: longitude value (in radians)
% 		
% 	Output:
% 		-[x, y, z]: an 1x3 vector with the resulting cartesian coordinates
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
		
    v = zeros(3, 1);
    l = cos(lat);
    v(1) = l*sin(long);
    v(2) = -sin(lat);
	v(3) = l*cos(long);

    if nargout == 1
        x = v;
    elseif nargout == 3
        x = v(1);
        y = v(2);
        z = v(3);
    end
end
