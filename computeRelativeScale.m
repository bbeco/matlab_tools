function relativeScale = computeRelativeScale(vSet, viewId, cameraParams, errorThreshold)
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
	
	% Selecting all the points that are visible in every views whose Ids are in
	% the vIds array
	matchNumber = 0;
	l = size(tracks, 2);
	points1 = zeros(l, 2, 'like', tracks(1, 1).Points(1, :));
	points2 = zeros(size(points1), 'like', points1);
	points3 = zeros(size(points1), 'like', points1);
	for i = 1:size(tracks, 2)
		if isequal(tracks(i).ViewIds, vIds)
			matchNumber = matchNumber + 1;
			points1(matchNumber, :) = tracks(i).Points(1, :);
			points2(matchNumber, :) = tracks(i).Points(2, :);
			points3(matchNumber, :) = tracks(i).Points(3, :);
		end
	end
	points1 = points1(1:matchNumber, :);
	points2 = points2(1:matchNumber, :);
	points3 = points3(1:matchNumber, :);
	
	camMatrix1 = cameraMatrix(cameraParams, camPoses.Orientation{1}, ...
		camPoses.Location{1});
	camMatrix2 = cameraMatrix(cameraParams, camPoses.Orientation{2}, ...
		camPoses.Location{2});
	camMatrix3 = cameraMatrix(cameraParams, camPoses.Orientation{3}, ...
		camPoses.Location{3});
	
	[prevWorldPoints, prevReprojectionErrors] = triangulate(points1, points2, camMatrix1, camMatrix2);
	[currWorldPoints, currReprojectionErrors] = triangulate(points2, points3, camMatrix2, camMatrix3);
	
	indexes = 1:size(prevWorldPoints, 1);
	
	% This creates all the possible combination of the indexes used for the
	% scale estimation.
	indexPairs = nchoosek(indexes, 2);
	
	l = size(indexPairs, 1);
	scaleEstimation = zeros(l, 1);
	
	numberOfEstimation = 0;
	for i = 1:l
		if prevReprojectionErrors(indexPairs(i, 1)) > errorThreshold || ...
				prevReprojectionErrors(indexPairs(i, 2)) > errorThreshold ||...
				currReprojectionErrors(indexPairs(i, 1)) > errorThreshold ||...
				currReprojectionErrors(indexPairs(i, 2)) > errorThreshold
			continue;
		end
		
		numberOfEstimation = numberOfEstimation + 1;
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
		
		scaleEstimation(numberOfEstimation) = num/den;
	end
	
	scaleEstimation = scaleEstimation(1:numberOfEstimation);
	relativeScale = median(scaleEstimation);
% 	relativeScale = mean(scaleEstimation);
end