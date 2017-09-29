function [u, v] = ll2equirectangular(lat, long, width, height)
	%LL2EQUIRECTANGULAR convert ll coordinates to image coordinates
	%   convert the LL coordinate to image coordinate for equirectangular
	%   mapping.
	u = min(width, ceil((long + pi)/(2*pi)*width));
	v = min(height, ceil((pi/2 - lat)/pi*height));
end

