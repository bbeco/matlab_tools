function poses = translateLocation(poses)
%	This function translates every poses so that the first camera's location 
%	is eye(3).
%	
%	Input:
%		-poses: the table with camera's locations and orientations.
%
%	Output:
%		poses: The rotated set of poses.
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
	t = poses.Location{1};
	
	if isequal(t, [0 0 0])
		return;
	end
	
	for i = 1:size(poses, 1)
		poses.Location{i} = poses.Location{i} - t;
	end
end
