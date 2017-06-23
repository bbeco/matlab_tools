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

I1 = imread('~/syntetic_scene2/0001.png');
I2 = imread('~/syntetic_scene2/0050.png');

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

%% Display results
% figure;
% ax = axes;
% showMatchedFeatures(I1, I2, matchedPts1, matchedPts2, 'montage', 'Parent', ax);
% title(ax, 'Candidate point matches');
p = [-1, -1];
while 1
	showMatches(I1, I2, matchedPts1, matchedPts2, p);
	p = ginput(1);
end

function showMatches(I1, I2, points1, points2, selectedPoint)
	subplot(2, 1, 1);
	I1 = insertMarker(I1, points1, 'o', 'Color', 'red');
	imshow(I1);
	
	subplot(2, 1, 2);
	imshow(I2);
	
	if selectedPoint(1) < 0 || selectedPoint(2) < 0
		return;
	end

	i = closestPoint(selectedPoint, points1.Location);

	subplot(2, 1, 1);
	hold on;
	x = points1.Location(i, :);
	plot(x(1), x(2),'g+');
	hold off;

	subplot(2, 1, 2);
	x = points2.Location(i, :);
	hold on;
	plot(x(1), x(2), 'go');
	hold off;
	
end

function index = closestPoint(p, v)
	[r, c] = size(v);
	min = sqrt(sum((p - v(1, :)).^2));
	index = 1;
	
	for i = 2:r
		d = sqrt(sum((p - v(i, :)).^2));
		if d < min
			min = d;
			index = i;
		end
	end
end
