function [xyzPoints, colors] = myTriangulateMidPoints(color1, disparityMap, rot1, rot2, loc1, loc2, orient1, orient2, minDisp, maxDistance)
%MYTRIANGULATEMIDPOINTS Summary of this function goes here
%   Detailed explanation goes here
	if ~exist('minDisp', 'var')
		minDisp = 1;
	end
	
	if ~exist('maxDistance', 'var')
		maxDistance = -1;
	end

	zMin = 0.01;
	
	[height, width, ~] = size(disparityMap(:,:,1));
	
	% do not triangulate points whose latitude absolute value is above this value
	lambdaMax = 60/180*pi;

	xyzPoints = zeros(height*width, 3);
	colors = zeros(height*width, 3, 'uint8');
	xyzSetLength = 0;
	
	relativeOrient = orient2*orient1';
	relativeLoc = (loc2 - loc1)*orient1';
	
	[R, t] = cameraPoseToExtrinsics(relativeOrient, relativeLoc);
	
	camMatrix1 = cameraMatrix(cameraParameters, eye(3), [0 0 0]);
	camMatrix2 = cameraMatrix(cameraParameters, R, t);
	
	maxDistance = norm(relativeLoc) * maxDistance;
	
	for v = 1:size(disparityMap, 1)
		for u = 1:size(disparityMap, 2)
			
			if disparityMap(v, u) < minDisp
				continue;
			end
			
			[latL, long] = extractLLCoordinateFromImage(u, v, width, height);
			[latR, ~] = extractLLCoordinateFromImage(u, v + disparityMap(v, u), ...
				width, height);
			
			%If the points is near the baseline (around the poles' areas in the
			%images, discard it
			if abs(latL) > lambdaMax || abs(latR) > lambdaMax
				continue;
			end
			
			dir1 = LL2Cartesian(latL, long);
			dir2 = LL2Cartesian(latR, long);
			
			point1 = rot1*dir1;
			point2 = rot2*dir2;
			
			if abs(point1(3)) < zMin || abs(point2(3)) < zMin
				continue;
			end
			
			point1 = point1/point1(3);
			point2 = point2/point2(3);
			
			point3D = triangulateMidPoint(point1(1:2)', point2(1:2)', camMatrix1, camMatrix2);
			if any(isnan(point3D)) || any(isinf(point3D))
				continue;
			end
			
			% discard points too far from the cameras
			distance = sqrt(sum(point3D.^2));
			if maxDistance > 0 && distance > maxDistance
				continue;
			end
			
			xyzSetLength = xyzSetLength + 1;
			xyzPoints(xyzSetLength, :) = point3D;
			% The color comes from the rectified color image
			colors(xyzSetLength, :) = color1(v, u, :);
		end
	end
	
	xyzPoints = xyzPoints(1:xyzSetLength, :);
	colors = colors(1:xyzSetLength, :);
end

%--------------------------------------------------------------------------
% Simple triangulation algorithm from
% Hartley, Richard and Peter Sturm. "Triangulation." Computer Vision and
% Image Understanding. Vol 68, No. 2, November 1997, pp. 146-157
function points3D = triangulateMidPoint(points1, points2, P1, P2)

numPoints = size(points1, 1);
points3D = zeros(numPoints, 3, 'like', points1);
P1 = P1';
P2 = P2';

M1 = P1(1:3, 1:3);
M2 = P2(1:3, 1:3);

c1 = -M1 \ P1(:,4);
c2 = -M2 \ P2(:,4);
y = c2 - c1;

u1 = [points1, ones(numPoints, 1, 'like', points1)]';
u2 = [points2, ones(numPoints, 1, 'like', points1)]';

a1 = M1 \ u1;
a2 = M2 \ u2;

isCodegen  = ~isempty(coder.target);
condThresh = eps(class(points1));

for i = 1:numPoints
    A   = [a1(:,i), -a2(:,i)];  
    AtA = A'*A;
    
    if rcond(AtA) < condThresh
        % Guard against matrix being singular or ill-conditioned
        p    = inf(3, 1, class(points1));
        p(3) = -p(3);
    else
        if isCodegen
            % mldivide on square matrix is faster in codegen mode.
            alpha = AtA \ A' * y;
        else
            alpha = A \ y;        
        end
        p = (c1 + alpha(1) * a1(:,i) + c2 + alpha(2) * a2(:,i)) / 2;
    end
    points3D(i, :) = p';

end
end