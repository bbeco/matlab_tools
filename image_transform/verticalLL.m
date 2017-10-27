function vImg = verticalLL(img)
	%VERTICALLL transform the LL image orientation from horizontal to vertical
	%
	[height, width] = size(img);
	
	vImg = zeros(width, height, 'like', img);
	for i = 1:width
		for j = 1:height
			vImg(width - i + 1, j) = img(j, i);
		end
	end
end