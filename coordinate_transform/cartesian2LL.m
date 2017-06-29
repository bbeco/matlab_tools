function [lat, long] = cartesian2LL(p)
%   Convert the given point to LL coordinates. The coordinate system of the
%   input is assumed to have x and y axes pointing right and up, and the -z
%   axe that points forward.
%   
%   Input:
%       -p: a 1x3 point vector
%   
%   Output:
%       -[lat, long]: 1x2 vector whose components are the latitude and 
%       longitude angles.
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



    long = arctan(-p(3), p(1));
    lat = arctan(sqrt(p(1)^2 + p(3)^2), p(2));
end