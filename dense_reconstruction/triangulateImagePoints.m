function [xyzPoints, colors] = triangulateImagePoints(color1, disparityMap, baseline, minDisp)
	%TRIANGULATEIMAGEPOINTS 
	
	%Input data to the densifiction function:
	% color1, color2, disparityMap, baseline.
	% minDisp (optional) do not triangulate points whose disparity is below this
	% value.
	%
	%Output by densifiction:
	%xyzPoints with color
	
	if ~exist('minDisp', 'var')
		minDisp = 0;
	end

	[height, width, ~] = size(color1);
	
	% do not triangulate points whose latitude absolute value is above this value
	lambdaMax = 60/180*pi;

	xyzPoints = zeros(height*width, 3);
	colors = zeros(height*width, 3, 'uint8');
	xyzSetLength = 0;

	for u = 1:size(disparityMap, 2)
		for v = 1:size(disparityMap, 1)
			% We can not estimate distance without disparity
			if disparityMap(v, u) <= minDisp
				continue;
			end

			[latL, long] = extractLLCoordinateFromImage(u, v, width, height);
			[latR, ~] = extractLLCoordinateFromImage(u, v + disparityMap(v, u), ...
				width, height);

			%If the points is near the baseline (around the poles' zones in the
			%images, discard it
			if abs(latL) > lambdaMax || abs(latR) > lambdaMax
				continue;
			end

			alpha = pi/2 - latL;
			beta = pi/2 - latR;

			depth = baseline*(sin(alpha)*sin(beta))/sin(beta - alpha);
			r = depth/sin(alpha);

			%computing point's coordinates
			p = r*LL2Cartesian(latL, long);

			xyzSetLength = xyzSetLength + 1;
			xyzPoints(xyzSetLength, :) = p;

			%extracting color
			colors(xyzSetLength, :) = color1(v, u, :);
		end
	end
	
	xyzPoints = xyzPoints(1:xyzSetLength, :);
	colors = colors(1:xyzSetLength, :);
end

