classdef ViewWindow < handle
	% Similar to ViewsSet for spherical images.
	% It is used to track points on a set of images.
	properties (SetAccess = private)
		Views		% table with information for each view
		WindowSize	% size of the window
		NumViews
	end
	methods
		function obj=ExtendedViewSet(num)
			obj.Views = table;
			obj.WindowSize = num;
			obj.NumViews = 0;
		end
		function addPoints(obj, viewId, points, features, conversion)
			if obj.NumViews < obj.WindowSize
				obj.NumViews = obj.NumViews + 1;
			end
			
			% shift
			if obj.NumViews > 1
				for i = obj.NumViews:-1:2
					obj.Views{i, :} = obj.Views{i - 1, :};
				end
			end
			% add new image
			obj.Views.ViewId{1} = viewId;
			obj.Views.Points{1} = points;
			obj.Views.Features{1} = features;
			obj.Views.Conversion{1} = conversion;
		end
	end
			
end