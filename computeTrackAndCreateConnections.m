function vSet = computeTrackAndCreateConnections(vSet, vWindow, lastViewsPairs)
%	This function finds point tracks among several views and stores the matches 
%	in the ViewSet's connections.
%
%	TODO: this should be improved by exploiting the matlab's intersect function 
%	and some kind of balanced binary tree for round based features comparison.
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

	correspondences = table(1, vWindow.WindowSize - 1);
	matches = table(1, vWindowSize - 1);
	matchesCounter = 0;
	correspondences(1, 1) = lastViewsPairs(:, [2, 1]);
	
	features1 = vWindow.Views.Features{1}
	for i = 3:vWindowSize
		features2 = vWindow.Views.Features{i};
		indexPairs = matchFeatures(features1, features2);
		correspondences(1, i - 1) = indexPairs;
	end

	v = zeros(1, vWindow.WindowSize - 1);
	viewsCounter = 1;
	for i = 1:height(lastViewsPairs)
		index = correspondences(1, 1);
		[~, ~, ib] = intersect(lastViewsPairs(i, 1), index(:, 2));
		v(1) = ib;
		viewsCounter = 2;
		while ~isempty(ib) && viewsCounter < vWindowSize
			index = correspondences(1, viewsCounter);
			[~, ~, ib] = intersect(lastViewPairs(i, 1), index(:, 2));
			v(viewsCounter) = ib;
			viewsCounter = viewsCounter + 1;
		end
		if viewsCounter >= vWindowSize
			for j = 1:length(v)
				matchesCounter = matchesCounter + 1;
				matches(matchesCounter, j) = v(j);
			end
		else
			continue;
		end
	end
		
	% Adding matches between views
	for i = 2:vWindowSize
		vSet = addConnection(vSet, 1, i, 'Matches', matches(i));
	end
	
end