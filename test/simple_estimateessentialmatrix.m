% 8 random points (homogeneous coordinate)
x = [-1, -1, 1, 1;
	1, -1, 1, 1;
	1, 1, 1, 1;
	-1, 1, 1, 1;
	0, 0, 2, 1;
	2, 0, 2, 1;
	2, 2, 2, 1;
	0, 2, 2, 1];
% scatter3(x(:,1), x(:, 2), x(:, 3), 'ro', 'filled');

% u0, v0 are chosen so that the final image coordinates are positive
u0 = 3;
v0 = 3;
P1 = [
	1,	0,	u0,	0;
	0,	1,	v0,	0;
	0,	0,	1,	0; ];

theta = -pi/6;
R = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
t = [1; 0; 0];
G = [R, t];
G(4, :) = [0 0 0 1];
% the second camera is translated along its x axis and rotated around y
P2 = P1*G;

points1 = zeros(size(x, 1), 2);
for i = 1:size(x, 1)
	m = P1*x(i,:)';
	u = m(1)/m(3);
	v = m(2)/m(3);
	z = 1;
	points1(i, :) = [u, v];
end

matched1 = SURFPoints(points1);

points2 = zeros(size(x, 1), 2);
for i = 1:size(x, 1)
	m = P2*x(i,:)';
	u = m(1)/m(3);
	v = m(2)/m(3);
	z = 1;
	points2(i, :) = [u, v];
end
% 
% P2*x'
matched2 = SURFPoints(points2);

cameraParams = cameraParameters('IntrinsicMatrix', P1(:, 1:3)');
[E, inliersIndex, status] = estimateEssentialMatrix(matched1, matched2, cameraParams);

[orientation, location, validPointsFraction] = relativeCameraPose(E, cameraParams, matched1, matched2)

	