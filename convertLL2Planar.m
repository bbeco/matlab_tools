function [Iup, Idown] = convertLL2Planar(LLImage)
	%backward warping
	[r, c] = size(LLImage);
	%the resulting images are square
	dim = min(r,c);
	Iup = zeros(dim,'uint8');
	Idown = zeros(dim, 'uint8');
	for i = 1:dim
		y = 2*i/dim - 1;
		for j = 1:dim
			x = 2*j/dim - 1;
			l = x^2 + y^2;
			if l > 1
				%this is the case when we are outside of the unit circle
				continue
			end
			
			long = arctan(x, y);
			z = sqrt(1 - l^2);
			lat = atan(z/l);
			queryPointYUp = (pi/2 - lat)*r/pi;
			% lat is inverted for this emisphere
			queryPointYDown = (pi/2 + lat)*r/pi;
			queryPointX = (long + pi)/(2*pi)*c;
			%Iup(i, j) = LLImage(max(1, floor(queryPointYUp)), max(1, floor(queryPointX)));
			Iup(i, j) = interp2(LLImage, queryPointX, queryPointYUp, 'linear');
			
			Idown(i, j) = interp2(LLImage, queryPointX, queryPointYDown, 'linear');
			%Idown(i, j) = LLImage(max(1,floor(queryPointYDown)), max(1,floor(queryPointX)));
		end
	end
end
