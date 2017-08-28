classdef ViewWindow < handle
	% Similar to ViewsSet for spherical images.
	% It is used to track points on a set of images.
	properties (SetAccess = private)
		Views		% table with information for each view
		WindowSize	% size of the window
		NumViews
		Connections % table with inter-views information (matches mainly)
	end
	methods
		function obj=ViewWindow(num)
			obj.Views = table;
			obj.WindowSize = num;
		end
		
		function addPoints(obj, viewId, points, features, conversion)
			
			view = table(viewId, {points}, {features}, {conversion}, ...
				'VariableNames', {'ViewId', 'Points', 'Features', ...
				'Conversion'});
			
			if isempty(obj.Views)
				obj.Views = view;
			else
				% add new view
				obj.Views = [obj.Views; view];
				if size(obj.Views, 1) > obj.WindowSize
					% shift
					obj.Views = obj.Views(2:end, :);
				end
			end
		end
		
		function addConnection(this, viewId1, viewId2, matches)
			
			connection = table(viewId1, viewId2, {matches}, ...
				{[]}, {[]}, ...
				'VariableNames', {'ViewId1', 'ViewId2', 'Matches',...
				'RelativeOrientation', 'RelativeLocation'});
			
			if isempty(this.Connections)
				this.Connections = connection;
			else
				this.Connections = [this.Connections; connection];
			end
		end
		
	end
			
end