function vSet = computeTrackAndCreateConnections(vSet, vWindow, lastViewPairs)
%	This function finds point tracks among several views and stores the matches 
%	in the ViewSet's connections.
%		Input:
%			-vSet: The viewSet to be used to store connections
%			-vWindow: The ViewWindow object uset to store matching
%				information between views inside the window
%			-lastViewPairs: the result of the last call to matchFeatures
%				(this are the matches between the last and second last
%				view);
%
%		Output:
%			-vSet: The viewSet updated with the newly discovered
%			connections.
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

	if isempty(lastViewPairs)
        % No matches in the last 2 views. There cannot be any longer
        % tracks
		warning(...
			'Unable to compute tracks: no matches in the last 2 views');
		return;
	end
	
	correspondences = cell(vWindow.WindowSize);
	correspondences{end - 1, end} = lastViewPairs;
	
	%This creates a table with all the mathces among every view contained
	%in the window. Only the upper left triangle block is relevant since
	%the matches are symmetric
	for i = 1:vWindow.WindowSize - 2
		features1 = vWindow.Views.Features{i};
		for j = (i + 1):vWindow.WindowSize
			features2 = vWindow.Views.Features{j};
			index = matchFeatures(features1, features2, 'Unique', true);
			correspondences{i, j} = index;
		end
	end
	
	goodMatches = cell(size(correspondences));
	v = zeros(1, vWindow.WindowSize);
	for i = 1:vWindow.WindowSize - 1
		len = size(correspondences{i, i + 1}, 1);
		if len < 1
			% It means we have just found two consecutive views without
			% any matches.
			id1 = vWindow.Views.ViewId(i);
			id2 = vWindow.Views.ViewId(i + 1);
			warning(['No correspondences between frame ', num2str(id1), ...
				' and frame ', num2str(id2)]);
			return;
		end
		for j = 1:len
			% reset view counter
			viewCounter = i + 1;
			while viewCounter <= vWindow.WindowSize
				%looking for correspondences with view (i + 1)
				index = correspondences{i, viewCounter}(:, 1);
				[~, ~, ib] = intersect(...
					correspondences{i, i + 1}(j, 1), index);
				if ~isempty(ib)
					v(viewCounter) = ib;
					if viewCounter >= vWindow.WindowSize
						% TODO check tracks coherence
						for k = (i + 1):vWindow.WindowSize
							goodMatches{i, k}(end + 1, :) = ...
								correspondences{i, k}(v(k), :);
						end
						% we exit from the inner loop for sure now
					end
					viewCounter = viewCounter + 1;
				else
					% this features is not found in every view of the
					% window
					break;
				end
			end
		end
	end
	
	% Add goodMatches only
	for i = 1:vWindow.WindowSize - 1
		for j = (i + 1):vWindow.WindowSize
			id1 = vWindow.Views.ViewId(i);
			id2 = vWindow.Views.ViewId(j);
			if ~hasConnection(vSet, id1, id2)
				vSet = addConnection(vSet, id1, id2, ...
					'Matches', goodMatches{i, j});
			else
				for k = 1:size(vSet.Connections, 1)
					if ~isempty(vSet.Connections) &&...
							vSet.Connections.ViewId1(k) == id1 ...
							&& vSet.Connections.ViewId2(k) == id2
						% Combine data and remove repetitions
						oldMatches = vSet.Connections.Matches{k};
						matches = union(oldMatches, goodMatches{i, j}, ...
							'rows');
						vSet = updateConnection(vSet, id1, id2, ...
							'Matches', matches);
					end
				end
			end
		end
	end
end