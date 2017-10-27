function rot = axisRot2mat(axis, theta)
	%AXISROT2MAT returns a rotation matrix that rotates a point by <theta> about
	%	<axis>.
	%	The rotation matrix has been taken from the book "Mathematics for 3D
	%	game programming and computer graphics", Third edition.
	c = cos(theta);
	s = sin(theta);
	a = axis;
	rot = [
		c+(1-c)*a(1)^2,	(1-c)*a(1)*a(2)-s*a(3)	(1-c)*a(1)*a(3)+s*a(2);
		(1-c)*a(1)*a(2)+s*a(3)	c+(1-c)*a(2)^2	(1-c)*a(2)*a(3)-s*a(1);
		(1-c)*a(1)*a(3)-s*a(2)	(1-c)*a(2)*a(3)+s*a(1)	c+(1-c)*a(3)^2
		];
	
end