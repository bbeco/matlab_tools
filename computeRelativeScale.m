function relativeScale = computeRelativeScale(vSet, viewId, cameraParams)
% Compute the relative scale as the median of all the scale computed between two
% point pairs.
% This function uses the view informations from the last three views.
%
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
	vIds = [viewId-2, viewId-1, viewId];
	camPoses = poses(vSet, vIds);
	tracks = findTracks(vSet, vIds);
	
	points1 = tracks(1).Points;
	points2 = tracks(2).Points;
	points3 = tracks(3).Points;
	
	camMatrix1 = cameraMatrix(cameraParams, camPoses.Orientation{1}, ...
		camPoses.Location{1});
	camMatrix2 = cameraMatrix(cameraParams, camPoses.Orientation{2}, ...
		camPoses.Location{2});
	camMatrix3 = cameraMatrix(cameraParams, camPoses.Orientation{3}, ...
		camPoses.Location{3});
	
	prevWorldPoints = triangulate(points1, points2, camMatrix1, camMatrix2);
	currWorldPoints = triangulate(points2, points3, camMatrix2, camMatrix3);
	
	indexes = 1:size(prevWorldPoints, 1);
	
	% This creates all the possible combination of the indexes used for the
	% scale estimation.
	indexPairs = nchoosek(index, 2);
	
	l = size(indexPairs, 1);
	scaleEstimation = zeros(l, 1);
	
	for i = 1:l
		num = sqrt(...
			sum(...
			(prevWorldPoints(indexPairs(i, 1)) - ...
			prevWorldPoints(indexPairs(i, 2))).^2) ...
			);
		
		den = sqrt(...
			sum(...
			(currWorldPoints(indexPairs(i, 1)) - ...
			currWorldPoints(indexPairs(i, 2))).^2) ...
			);
		
		scaleEstimation(i) = num/den;
	end
	
	relativeScale = median(scaleEstimation);
end