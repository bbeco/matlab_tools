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
I1 = imread('images/essential_matrix_test/ll1.png');
I2 = imread('images/essential_matrix_test/ll2.png');
I1 = rgb2gray(I1);
I2 = rgb2gray(I2);

% Extracting interest points
points1 = detectSURFFeatures(I1);
points2 = detectSURFFeatures(I2);
% points1 = detectHarrisFeatures(I1);
% points2 = detectHarrisFeatures(I2);

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
% figure;
% ax = axes;
% showMatchedFeatures(I1, I2, matchedPts1, matchedPts2, 'montage', 'Parent', ax);
% title(ax, 'Candidate point matches');
addpath('utils/');
% p = [-1, -1];
% while 1
% 	showMatches(I1, I2, matchedPts1, matchedPts2, p);
% 	p = ginput(1);
% end

%% project the LL coordinate on a plane
if size(I1) ~= size(I2)
	error('The size of the input images are different');
end

[height, width] = size(I1);
dim = min(height, width);
% Optical center coordinates on the image plane
u0= dim/2;
v0 = u0;

frontCount = 0;
backCount = 0;
addpath('coordinate_transform/');
for i = 1:length(matchedPts1)
	% 
	LLu1 = matchedPts1.Location(i, 1);
	LLv1 = matchedPts1.Location(i, 2);
	long = LLu1/width*2*pi - pi;
	lat = pi/2 - LLv1/height*pi;
	[u1, v1, z1] = projectLL2Plane(lat, long, dim);
	
	%
	LLu2 = matchedPts2.Location(i, 1);
	LLv2 = matchedPts2.Location(i, 2);
	long = LLu2/width*2*pi - pi;
	lat = pi/2 - LLv2/height*pi;
	[u2, v2, z2] = projectLL2Plane(lat, long, dim);
	
	if z1 >= 0 && z2 >= 0
		frontCount = frontCount + 1;
		frontEmisphereMatches1(frontCount, :) = [u1, v1];
		frontEmisphereMatches2(frontCount, :) = [u2, v2];
	elseif z1 < 0 && z2 < 0
		backCount = backCount+ 1;
		backEmisphereMatches1(backCount, :) = [u1, v1];
		backEmisphereMatches2(backCount, :) = [u2, v2];
	end
end

%selecting the emisphere with more feature matches
if frontCount >= backCount
	emisphereMatches1 = SURFPoints(frontEmisphereMatches1);
	emisphereMatches2 = SURFPoints(frontEmisphereMatches2);
	disp([num2str(frontCount), ' matches in the front emisphere']);
else
	emisphereMatches1 = SURFPoints(backEmisphereMatches1);
	emisphereMatches2 = SURFPoints(backEmisphereMatches2);
	disp([num2str(backCount), ' matches in the back emisphere']);
end

%% compute E
%initializing camera parameters;
K = eye(3);
K(:, 3) = [u0; v0; 1];
cameraParams = cameraParameters('IntrinsicMatrix', K');
[E, inliersIndex, status] = estimateEssentialMatrix(emisphereMatches1, emisphereMatches2, cameraParams);
if status ~= 0
	error('some error occurred');
end

inliersPoints1 = emisphereMatches1(inliersIndex);
inliersPoints2 = emisphereMatches2(inliersIndex);

[orientation, location, validPointsFraction] = relativeCameraPose(E, cameraParams, inliersPoints1, inliersPoints2)

%% Checking epipolar contraint
epipolarConstraintError = zeros(length(inliersPoints1), 1);
for i = 1:length(inliersPoints1)
	m1 = inliersPoints1.Location(i, :);
	m1(3) = 1;
	p1 = K/m1;
	m2 = inliersPoints2.Location(i, :);
	m2(3) = 1;
	p2 = K/m2;
	epipolarConstraintError(i) = abs(p2'*E*p1);
end
mean(epipolarConstraintError)
