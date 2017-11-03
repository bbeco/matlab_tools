function [u, v] = ll2equirectangular(lat, long, width, height)
	%LL2EQUIRECTANGULAR convert ll coordinates to image coordinates
	%   convert the LL coordinate to image coordinate for equirectangular
	%   mapping.
	u = max(1, floor((long + pi)/(2*pi)*(width -1)) + 1);
	v = max(1, floor((pi/2 - lat)/pi*(height - 1)) + 1);
% 	u = (long + pi)/(2*pi)*(width -1) + 1;
% 	v = (pi/2 - lat)/pi*(height - 1) + 1;
end

