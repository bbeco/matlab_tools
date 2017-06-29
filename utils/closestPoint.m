function index = closestPoint(p, v)
	[r, c] = size(v);
	min = sqrt(sum((p - v(1, :)).^2));
	index = 1;
	
	for i = 2:r
		d = sqrt(sum((p - v(i, :)).^2));
		if d < min
			min = d;
			index = i;
		end
	end
end