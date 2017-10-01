function patches = createPatch(llImg, plat, plong, llwidth, llheight)
	%CREATEPATCH Compute the image patch for window matching algorithm
	%   This function projects an equirectangular image's area into a window
	%   (patch). The patch represents an input suitable for block matching
	%   algorithm.
	%
	%	Input:
	%		-llImg: the equirectangular image;
	%		-plat: a 1-by-N vector of latitude coordinates of the patch's 
	%			center;
	%		-plong: the longitude coordinate of the patch's center.
	%
	%	Output:
	%		-patches: an 1-by-N cell array of patches, one for each LL
	%			coordinate provided.
	%
	%	Copyright 2017 Andrea Beconcini
	%
	%	This program is free software: you can redistribute it and/or modify
	%	it under the terms of the GNU Lesser General Public License as published
	%	by the Free Software Foundation, either version 3 of the License, or
	%	any later version.
	% 
	%	This program is distributed in the hope that it will be useful,
	%	but WITHOUT ANY WARRANTY; without even the implied warranty of
	%	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	%	GNU Lesser General Public License for more details.
	% 
	%	You should have received a copy of the GNU Lesser General Public License
	%	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	%

	% The patch size in the same unit of the 3D sphere radius.
	patchSize = 1;
	
	% The patch size in pixel
	uMax = 601;
	vMax = 601;
	
	% The patch's principal point
	u0 = ceil(uMax/2);
	v0 = ceil(vMax/2);
	
	% Patch's pixel density
	k_u = patchSize/uMax;
	k_v = patchSize/vMax;
	
	patches = cell(1, length(plong));
	
	for k = 1:length(plong)
		patches{k} = zeros(vMax, uMax, 'like', llImg);
		
		% find the u and v directions that lie on the patch's plane
		r = LL2Cartesian(plat, plong(k));
		vdir = zeros(3, 1);
% 		if r(2) ~= 0
% 			udir = [1, -r(1)/r(2), 0];
% 		else
% 			udir = [1, 0, 0];
% 		end
% 		vdir(3) = 1;
% 		vdir(2) = -vdir(3)/(1 - udir(2)/udir(1))*r(2);
% 		vdir(1) = -udir(2)/udir(1)*vdir(2);
		
		% setting the u and v vector to be the pixel size
% 		udir = udir/norm(udir);
% 		vdir = vdir/norm(vdir);
		if plat == 0
			vdir = [0 1 0]';
		elseif plat == pi/2 || plat == -pi/2
			vdir = [0 0 -1]';
		else
			vdir(1) = 1;
			vdir(3) = r(3)/r(1);
			vdir(2) = -(r(1) + r(3)^2/r(1))/r(2);
			vdir = vdir/norm(vdir);
		end
		
		if abs(plong) == pi/2 || plong == 0
			udir = [1 0 0]';
			vdir = [0 1 0]';
		else
			udir = [1, 0, -r(1)/r(3)]';
			udir = udir/norm(udir);
		end
		
		for v = 1:vMax
			for u = 1:uMax
				p = r + k_u*(u - u0)*udir + k_v*(v - v0)*vdir;
				[lat, long] = cartesian2LL(p);
				[llu, llv] = ll2equirectangular(lat, long, llwidth, llheight);
				patches{k}(v, u) = llImg(llv, llu);
			end
		end
	end
end

