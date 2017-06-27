[matchedPoints1, matchedPoints2] = function findCorrespondances(I1, I2)
%	This function detects keypoints in both input images and find 
%	correspondances among them.
%
%	It extracts SURF features and their descriptors. It performs exhaustive 
%	matching among with default threshold and ratios for ambiguous matches. 
%	Every correspondance between the first image and the second is 
%	forward-backward checked.
%
%	Input:
%		-I1: the first image
%		-I2: the second image
%
%	Output:
%		-[matchedPoints1, matchedPoints2]: this are 2 array of matched 
%			points. At the i-th row in matchedPoints1 there is the 
%			point that matches with the point at the same row in 
%			matchedPoints2.
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

	% Extracting interest points
	points1 = detectSURFFeatures(I1);
	points2 = detectSURFFeatures(I2);

	% Extract interesting point descriptors
	[features1, vpts1] = extractFeatures(I1, points1);
	[features2, vpts2] = extractFeatures(I2, points2);

	% Feature points matching uses:
	%	Exhaustive, all pairwise checks are performed;
	%	Default match threshold, 1.0 for SURF
	%	Default maxRatios, 0.6 for ambiguous matches
	%	Default matching metric, SAD
	%	Mutual Consistency check, forward-backward check between features1 and 2
	indexPairs = matchFeatures(features1, features2, 'Unique', true);

	% Selecting matched points only
	matchedPts1 = vpts1(indexPairs(:, 1));
	matchedPts2 = vpts2(indexPairs(:, 2));
end
