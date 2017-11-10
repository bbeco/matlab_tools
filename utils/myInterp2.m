function v = myInterp2(gray, u, v, width, height)
%	MYINTERP2 bilinear interpolation for grayscale images
%
%	This is faster than MATLAB's interp2 but it only works for single channel
%	grayscale images.
%
%	Input:
%		-gray: grayscale input image
%		-u, v: the coordinates of the query point 1<= u <=width and
%		1<= v <=height;
%		-height, width: image's width and height.
%
%	Output:
%		-v: interpolated value;
%
	if v == height
		y1 = height - 1;
		y2 = height;
	else
		y1 = floor(v);
		y2 = y1 + 1;
	end
	
	if u == width
		x1 = width - 1;
		x2 = width;
	else
		x1 = floor(u);
		x2 = x1 + 1;
	end
	
	Q = [	gray(y1, x1), gray(y2, x1);
		gray(y1, x2), gray(y2, x2)];
	v = [x2 - u, u - x1] * Q * [y2 - v; v - y1]; 
end
