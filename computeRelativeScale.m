function relativeScale = computeRelativeScale(vSet, viewId, cameraParams, ...
		errorThreshold)
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
	camPoses = poses(vSet);
	
	loc1 = camPoses.Location{viewId - 2};
	orient1 = camPoses.Orientation{viewId - 2};
	[R1, t1] = cameraPoseToExtrinsics(orient1, loc1);
	camMatrix1 = cameraMatrix(cameraParams, R1, t1);
	
	loc2 = camPoses.Location{viewId - 1};
	orient2 = camPoses.Orientation{viewId - 1};
	[R2, t2] = cameraPoseToExtrinsics(orient2, loc2);
	camMatrix2 = cameraMatrix(cameraParams, R2, t2);
	
	currLoc = camPoses.Location{viewId};
	currOrient = camPoses.Orientation{viewId};
	[R, t] = cameraPoseToExtrinsics(currOrient, currLoc);
	currCamMatrix = cameraMatrix(cameraParams, R, t);
	
	% Selecting all the points that are visible in every views whose Ids are in
	% the vIds array
	prevMatchesIdx = vSet.Connections.Matches{end - 1};
	currMatchesIdx = vSet.Connections.Matches{end};
	
	[~, ia, ib] = intersect(prevMatchesIdx(:, 2), currMatchesIdx(:, 1));
	idx1 = prevMatchesIdx(ia, 1);
	idx2 = prevMatchesIdx(ia, 2);
	currIdx = currMatchesIdx(ib, 2);
	
	points1 = vSet.Views.Points{viewId - 2};
	points2 = vSet.Views.Points{viewId - 1};
	currPoints = vSet.Views.Points{viewId};
	
	points1 = points1(idx1, :);
	points2 = points2(idx2, :);
	currPoints = currPoints(currIdx, :);
	
	[prevWorldPoints, prevReprojectionErrors] = triangulate(points1, points2,...
		camMatrix1, camMatrix2);
	[currWorldPoints, currReprojectionErrors] = triangulate(points2, ...
		currPoints,	camMatrix2, currCamMatrix);
	
	% This creates all the possible combination of the indexes used for the
	% scale estimation.
	l = size(prevWorldPoints, 1);
	if l < 2
		warning(['Unable too compute relative scale: too few points']);
		relativeScale = 1;
		return;
	end
	
	indexPairs = nchoosek(1:l, 2);
	
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
end