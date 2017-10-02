function disparityMap = computeDisparityEquirectangular(imgL, imgR, dm_patchSize, dm_maxDisparity, dm_regularization)
%COMPUTEDISPARITYEQUIRECTANGULAR Compute disparity map for an LL image pair
%   This function is inspired by computeDisparitySlow of HDR Toolbox.
	if(~exist('dm_patchSize', 'var'))
		dm_patchSize = 7;
	end

	if(~exist('dm_maxDisparity', 'var'))
		dm_maxDisparity = -1;
	end

	if(~exist('dm_regularization', 'var'))
		dm_regularization = 0.2;
	end

	if(dm_maxDisparity < 0.0)

		hf = figure(1);
		imshow((imgL + imgR) / 2.0);
		hold on;
		[x0, y0] = ginput(1);
		plot(x0, y0, 'r+');
		[x1, y1] = ginput(1);
		plot(x1, y1, 'r+');

		dm_maxDisparity = round(abs(x1 - x0));
		hold off;

		disp(dm_maxDisparity);
		close(hf);
	%    dm_maxDisparity = dm_patchSize * 4;    
	end
	
	[r, c, col] = size(imgL);
	
	if col > 1
		imgL = rbg2gray(imgL);
		imgR = rgb2gray(imgR);
	end
	
	disparityMap = zeros(r, c, 2);
	for u = 1:c
		disp(['Processing column: ', num2str(u), '/', num2str(c)]);
		%once we now the equipolar line, we can extract all the pathces
		%from the other image
		[latR, longR] = extractLLCoordinateFromImage(u, 1:r, c, r);
		[patchesR, patchesR_sq] = createPatch(imgR, latR, longR, c, r);
		for v = 1:r
			
			[latL, longL] = extractLLCoordinateFromImage(u, v, c, r);
			[patchL, patchL_sq] = createPatch(imgL, latL, longL, c, r);
			
			min_v = v - dm_maxDisparity;
			max_v = v + dm_maxDisparity;
			lambda = dm_regularization / (max_v - min_v + 1);
			err = 1e30;
			depth = 0;
			
			for k = min_v:max_v
				k = mod(k, r) + 1;
				delta = patchL{1} .* patchesR{k} / (sum(patchesR_sq{k}(:)) * sum(patchL_sq{1}(:)));
				
				tmp_err = mean(delta(:));
				d3 = k - v;
				if dm_regularization > 0 && u > 1 && v > 1
					d1 = disparityMap(v - 1, u, 1);
					d2 = disparityMap(v, u - 1, 1);
					
					tmp_err = tmp_err + lambda * (abs(d3) + abs(d3 - d1) + abs(d3 - d2) );
				end
				
				if tmp_err < err
					err = tmp_err;
					depth = d3;
				end
			end
			
			disparityMap(v, u, 1) = depth;
			disparityMap(v, u, 2) = err;
		end
	end
end

