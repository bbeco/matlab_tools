function camPoses = alignCameraPoses(camPoses)
%	This utility function aligns the camera poses so that the first view
%	orientation is the identity matrix.
%
% 		Input:
% 			camPoses: a table with ViewId, Orientation and Location informations
% 	
% 		Output:
% 			-camPoses: the modified camera pose set
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
	R = camPoses.Orientation{1}';
	for i = 1:height(camPoses)
		camPoses.Orientation{i} = camPoses.Orientation{i} * R;
	end
end
	