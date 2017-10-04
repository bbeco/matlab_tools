function disparityMap = computeDisparityEquirectangular(imgL, imgR, dm_patchSize, dm_maxDisparity, dm_regularization, column)
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
	
	% Debugging purposes
	if ~exist('column', 'var')
		min_u = dm_patchSize + 1;
		max_u = c - dm_patchSize - 1;
	else
		min_u = column;
		max_u = column;
	end
	
	if col > 1
		imgL = rbg2gray(imgL);
		imgR = rgb2gray(imgR);
	end
	
	disparityMap = zeros(r, c, 2);
	for u = min_u:max_u
		disp('Extracting patches');
		%once we now the epipolar line, we can extract all the patches
		%from the other image
		[latR, longR] = extractLLCoordinateFromImage(u, 1:r, c, r);
		[patchesR, patchesR_sq] = createPatch(imgR, latR, longR, c, r);
		disp(['Processing column: ', num2str(u), '/', num2str(c)]);
		for v = (dm_patchSize + 1):(r - dm_patchSize - 1)
% 			disp(['Processing row: ', num2str(v), '/', num2str(r)]);
			[latL, longL] = extractLLCoordinateFromImage(u, v, c, r);
			[patchL, patchL_sq] = createPatch(imgL, latL, longL, c, r);

			d1 = disparityMap(v - 1, u, 1);
			d2 = disparityMap(v, u - 1, 1);
			
			min_v = max([v - dm_maxDisparity, dm_patchSize + 1]);
			max_v = min([v + dm_maxDisparity, r - dm_patchSize - 1]);
			
			lambda = dm_regularization / (max_v - min_v + 1);
			err = 1e30;
			depth = 0;
			
			for k = min_v:max_v
				delta = patchL{1} .* patchesR{k} / (sum(patchesR_sq{k}(:)) * sum(patchL_sq{1}(:)));
				
				tmp_err = mean(delta(:));
				d3 = k - v;
					
				tmp_err = tmp_err + lambda * (abs(d3) + abs(d3 - d1) + abs(d3 - d2) );
				
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

