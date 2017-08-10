classdef ViewWindow < handle
	% Similar to ViewsSet for spherical images.
	% It is used to track points on a set of images.
	properties (SetAccess = private)
		Views		% table with information for each view
		WindowSize	% size of the window
		NumViews
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
	end
			
end