function [lat, long] = extractLLCoordinateFromImage(u, v, width, height)
%	Extracts the LL coordinates from an equirectangular image (Lat/Long
%	image).
%       Input:
%           -u: the horizontal image coordinate;
%           -v: the vertical image coordinate;
%           -width: the image width;
%           -height: the image height.
%
%       Output:
%           -lat: the latitude coordinate;
%           -long: the longitude coordinate.
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
    long = u/width*2*pi - pi;
    lat = pi/2 - v/height*pi;
end