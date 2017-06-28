%	Test script for features matching
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

%% find features
I1 = imread('images/img1_planar.png');
I2 = imread('images/img2_planar.png');

% Extracting interest points
points1 = detectSURFFeatures(I1);
points2 = detectSURFFeatures(I2);

subplot(2, 2, 1);
showFeaturePoints(I1, points1);
title('Features found in first image');
subplot(2, 2, 2);
showFeaturePoints(I2, points2);
title('Features found in second image');

% Extract interesting point descriptors.
% vpts stands for ValidPoinTS and is the name of the vector of keypoints whose 
% descriptor could be computed (because they are not too close to the edge)
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

subplot(2, 2, 3);
showFeaturePoints(I1, matchedPts1);
title('feature of image1 with a correspondance in 2');
subplot(2, 2, 4);
showFeaturePoints(I2, matchedPts2);
title('feature of image2 with a correpondance in 1');

%% Display results
figure;
ax = axes;
showMatchedFeatures(I1, I2, matchedPts1, matchedPts2, 'montage', 'Parent', ax);
title(ax, 'Candidate point matches');
% p = [-1, -1];
% while 1
% 	showMatches(I1, I2, matchedPts1, matchedPts2, p);
% 	p = ginput(1);
% end

%% compute E
cameraParams = cameraParameters;
[E, inliersIndex, status] = estimateEssentialMatrix(matchedPts1, matchedPts2, cameraParams);
if status ~= 0
	disp('some error occurred');
end

inliersPoints1 = matchedPts1(inliersIndex);
inliersPoints2 = matchedPts2(inliersIndex);

[orientation, location] = relativeCameraPose(E, cameraParams, inliersPoints1, inliersPoints2)
