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
	
	% pre-computing the distance from the sphere's center to each patch's pixel.
	distance = zeros(1, vMax);
	% TODO optimize by reducing the distance vector length to one half (becouse
	% of simmetry)
	for v = 1:vMax
		distance(v) = sqrt(1 + k_v^2*(v0 - v)^2);
	end
	
	for k = 1:length(plong)
		patches{k} = zeros(vMax, uMax, 'like', llImg);
		for v = 1:vMax
			lat = plat + atan(k_v*(v0 - v));
			if lat > 1.5708
				lat = lat - 1.5708;
			elseif lat < -1.5708
				lat = 0.0584 + lat;
			end
			for u = 1:uMax
				long = plong + atan(k_u*(u - u0)/(cos(lat)*distance(v)));
				p = LL2Cartesian(lat, long);
				[llat, llong] = cartesian2LL(p);
				% TODO change this with bilinear interpolation
				[llu, llv] = ll2equirectangular(lat, long, llwidth, llheight);
				patches{k}(v, u) = llImg(llv,  llu);
			end
		end
	end
end

