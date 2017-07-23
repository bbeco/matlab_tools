function v = reorderVector(u, index)
% This function orders the vector u as specified by the index vector.
% The index vector is scanned from beginning and the i-th element in u is moved
% to the position contained in the i-th element in index.
% This function is the inverse ordering performed by u = v(index);
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
	l = size(u, 1);
	if l ~= size(index, 1)
		error('Input vectors must be of the same length');
	end
	
	v = zeros(size(u), 'like', u);
	for i = 1:l
		v(index(i), :) = u(i, :);
	end
end