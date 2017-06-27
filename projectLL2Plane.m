[u, v] = function projectLL2Plane(lat, long)
	m = cos(lat);
	x = m*sin(long);
	y = m*cos(long);
	z = sin(lat);
