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
				index = correspondences{i, viewCounter};
				[~, ~, ib] = intersect(...
					correspondences{i, i + 1}(j, 1), index(:, 1));
				
				if ~isempty(ib)
					
					% checking if there is an existing path for features
					% correspondences{i, viewCounter}(ib, 1) from view
					% i to view viewCounter.
					idx = findMatchingFeatureIndex(correspondences, i, viewCounter,...
						index(ib, 1));
					if idx ~= index(ib, 2)
						vId1 = vWindow.Views.ViewId(i);
						vId2 = vWindow.Views.ViewId(viewCounter);
						warning([...
							'Features ', num2str(index(ib, 1)),...
							' in view ', num2str(vId1), ...
							' cannot be tracked to feature ', ...
							num2str(index(ib, 2)), ...
							' in view ', num2str(vId2)]);
						break;
					end
					
					v(viewCounter) = ib;
					if viewCounter >= vWindow.WindowSize
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

function destFeatureIndex = findMatchingFeatureIndex(correspondences, ...
	sourceView, destView, sourceFeatureIndex)
% This function follows the track for a specific feature index in
% sourceView view and return the destination feature index in destView
% view.
% If the there is no track for the input features that starts in sourceView
% and ends in destView, this function returns -1.
% Both sourceView and destView are not unique view ids, they are
% the sourceView's and destView's position inside the tracking window; e.g.
% sourceView = 1 for the oldest view in the window, 2 for the second oldest
% and so on.
	if sourceView >= destView || destView > size(correspondences, 2)
		error(['Unable to follow features from view ', ...
			num2str(sourceView), ' to view ', num2str(destView),...
			' in tracking window']);
	end
	
	l = size(correspondences{sourceView, sourceView + 1}, 1);
	for k = 1:l
		if correspondences{sourceView, sourceView + 1}(k, 1) == ...
				sourceFeatureIndex
			if sourceView + 1 == destView
				destFeatureIndex = ...
					correspondences{sourceView, sourceView + 1}(k, 2);
			else
				destFeatureIndex = findMatchingFeatureIndex(...
					correspondences, sourceView + 1, destView, ...
					correspondences{sourceView, sourceView + 1}(k, 2));
			end
			return;
		end
	end
	destFeatureIndex = -1;
end