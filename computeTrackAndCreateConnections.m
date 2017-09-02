function vSet = computeTrackAndCreateConnections(vSet, vWindow)
%	This function finds point tracks among several views and stores the matches 
%	in the ViewSet's connections.
%		Input:
%			-vSet: The viewSet to be used to store connections
%			-vWindow: The ViewWindow object uset to store matching
%				information between views inside the window
%
%		Output:
%			-vSet: The viewSet updated with the newly discovered
%			connections.
%
%	TODO: this should be improved by reusing already computed matches between
%	consecutive images
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

	correspondences = cell(vWindow.WindowSize - 1, vWindow.WindowSize);
	goodMatches = cell(size(correspondences));

	%This creates a table with all the mathces among every view contained
	%in the window. Only the upper left triangle block is relevant since
	%the matches are symmetric
	
	% fill the elements right above the diagonal
	for i = 1:vWindow.WindowSize - 1
		vId1 = vWindow.Views.ViewId(i);
		vId2 = vWindow.Views.ViewId(i + 1);
		connIdx = getConnectionIndex(vWindow, vId1, vId2);
		if connIdx <= 0
			warning(['No correspondences between frame ', num2str(vId1), ...
				' and frame ', num2str(vId2)]);
			return;
		end
		matches = vWindow.Connections.Matches{connIdx};
		%matching between features that are present in the previous views too
		if i > 1 && ~isempty(matches)
			[~, ~, ib] = intersect(correspondences{i - 1, i}(:, 2), ...
				matches(:, 1));
			matches = matches(ib, :);
		end
		if isempty(matches)
			id1 = vWindow.Views.ViewId(i);
			id2 = vWindow.Views.ViewId(i + 1);
			warning(['No correspondences between frame ', num2str(id1), ...
				' and frame ', num2str(id2)]);
			return;
		end
		correspondences{i, i + 1} = matches;
	end
	
	% fill all remainin elements
	for i = 1:vWindow.WindowSize - 1
		matches = correspondences{i, i + 1};
		for k = 1:size(matches, 1)
			v = zeros(1, vWindow.WindowSize);
			viewCounter = i + 1;
			feature1 = vWindow.Views.Features{i}(matches(k, 1), :);
			while viewCounter <= vWindow.WindowSize
				idx = findMatchingFeatureIndex(correspondences, i,...
					viewCounter, matches(k, 1));
				if idx > 0
					% TrackConsistency check
					feature2 = vWindow.Views.Features{viewCounter}(idx, :);
					trackConsistency = ~isempty(...
						matchFeatures(feature1, feature2, 'Unique', true));
					if trackConsistency
						v(viewCounter) = idx;
						viewCounter = viewCounter + 1;
					else
						%track consistency failed
						break;
					end
				else
					% feature not found in view viewCounter
					break;
				end
			end
			
			% The program arrives here if either the feature could not be
			% tracked till the lasts view, some track consistency check failed
			% or the feature has been tracked along every view. It depends on
			% viewCounter value
			if viewCounter > vWindow.WindowSize
				for j = (i + 1):vWindow.WindowSize
					goodMatches{i, j} = ...
						[goodMatches{i, j}; [matches(k, 1), v(j)]];
				end
			else
				% The track is not complete or it does not satisfy the track
				% consistency check, discard it
				continue;
			end
		end
		
		% If no tracks have been added, there is nothing else to do
		if isempty(goodMatches{i, i + 1})
			id1 = vWindow.Views.ViewId(i);
			id2 = vWindow.Views.ViewId(i + 1);
			warning(['No correspondences between frame ', num2str(id1), ...
				' and frame ', num2str(id2)]);
			return;
		end
		
		% The set of matches between the next two views have to be a subset of
		% the track computed in the previous iteration
		if i + 1 < vWindow.WindowSize
			[~, ~, ib] = intersect(...
				goodMatches{i, i + 1}(:, 2), ...
				correspondences{i + 1, i + 2}(:, 1));
			correspondences{i + 1, i + 2} = ...
				correspondences{i + 1, i + 2}(ib, :);
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

function matches = getMatches(vSet, vId1, vId2)
% This function return the matches stored in the connection between view vId1
% and vId2.
	matches = [];
	if ~hasConnection(vSet, vId1, vId2)
		return;
	end
	for j = 1: size(vSet.Connections, 1)
		if vSet.Connections.ViewId1 == vId1 && ...
			vSet.Connections.ViewId2 == vId2
			matches = vSet.Connections.Matches{id1, id2};
			break;
		end
	end
end

function index = getConnectionIndex(vWindow, viewId1, viewId2)
% This function looks for an entry in the ViewWindow's connection tables with
% the given view IDs. If such an entry is found, its index is returned,
% otherwise -1 is returned.
	if isempty(vWindow.Connections)
		index = -1;
		return;
	end
	
	for i = 1:height(vWindow.Connections)
		if vWindow.Connections.ViewId1(i) == viewId1 && ...
				vWindow.Connections.ViewId2(i) == viewId2
			index = i;
			return;
		end
	end
	index = -1;
end
