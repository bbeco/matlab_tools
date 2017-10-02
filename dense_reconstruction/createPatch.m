function [patches, patches_sq] = createPatch(llImg, plat, plong, llwidth, llheight)
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
	patchSize = 0.1;
	
	% The patch size in pixel
	uMax = 7;
	vMax = 7;
	
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
		vdir = [0 1 0]';
		udir = [1 0 0]';
		
		% computing the rotation for the u and v directions
		a = LL2Cartesian(0, plong(k) + pi/2);
		rot = axisRot2mat(a, plat);
		
		% we have to rotate udir about the y axis also by plong(k)
		udir = rot * eul2rotm([0 plong(k) 0]) * udir;
		vdir = rot * vdir;
		
		for v = 1:vMax
			for u = 1:uMax
				p = r + k_u*(u - u0)*udir + k_v*(v - v0)*vdir;
				[lat, long] = cartesian2LL(p);
				[llu, llv] = ll2equirectangular(lat, long, llwidth, llheight);
				patches{k}(v, u) = llImg(llv, llu);
			end
		end
	end
	
	if nargout > 1
		patches_sq = cell(1, length(plong));
		for k = 1:length(plong)
			patches_sq{k} = patches{k}.^2;
		end
	end
end

