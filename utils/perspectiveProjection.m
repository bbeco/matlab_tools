function m = perspectiveProjection(M, f, u0, v0)
	P = [f, 0, u0, 0;
		0, f, v0, 0;
		0, 0, f, 0];
	l = size(M, 1);
	m = zeros(l, 2);
	for i = 1:l
		x = M(i, :);
		x(4) = 1;
		image_point = P*x';
		m(i, :) = image_point(1:2)/image_point(3)';
	end
end
		
		