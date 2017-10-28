function [disparityMap, dm_maxDisparity] = computeDisparityEquirectangular(imgL, imgR, dm_patchSize, dm_maxDisparity, dm_regularization, dm_alpha, column)
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
	
	if(~exist('dm_alpha', 'var'))
		dm_alpha = 0.05;
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
	
    dm_alpha_inv = (1.0 - dm_alpha);
    
	disparityMap = zeros(r, c, 2);
	%temporary variables to use parfor
	depthMap = zeros(r, c);
	errorMap = zeros(r, c);
	parfor u = min_u:max_u
		%once we know the epipolar line, we can extract all the patches
		%from the other images
		[latR, longR] = extractLLCoordinateFromImage(u, 1:r, c, r);
		[patchesR, ~, patchesR_dx] = createPatch(imgR, latR, longR, c, r, ...
			dm_patchSize);
		disp(['Processing column: ', num2str(u), '/', num2str(c)]);
		for v = (dm_patchSize + 1):(r - dm_patchSize - 1)
% 			disp(['Processing row: ', num2str(v), '/', num2str(r)]);
			[latL, longL] = extractLLCoordinateFromImage(u, v, c, r);
			[patchL, ~, patchL_dx] = createPatch(imgL, latL, longL, c, r, ...
				dm_patchSize);

			%removed to use parfor
% 			d1 = disparityMap(v - 1, u, 1);
% 			d2 = disparityMap(v, u - 1, 1);
			
			min_v = max([v - dm_maxDisparity, dm_patchSize + 1]);
			max_v = min([v + dm_maxDisparity, r - dm_patchSize - 1]);
			
            % if we use the sum of the SSD response, decomment this
            % otherwise leave it as it is.
			%lambda = dm_regularization / (max_v - min_v + 1);
%             lambda = dm_regularization;
			err = 1e30;
			depth = 0;
			
			for k = min_v:max_v
				%SSD
				delta = (patchL{1} - patchesR{k}).^2;
				
				% gradient matching
                delta_dx_sq = (patchL_dx{1} - patchesR_dx{k}).^2;
                
				tmp_err = dm_alpha_inv * sum(delta(:)) + dm_alpha * sum(delta_dx_sq(:));
				% simplified formula
%  				tmp_err = sum(delta(:));
				d3 = k - v;
				
				% removed just like regularization
				%tmp_err = tmp_err + lambda * (abs(d3) + abs(d3 - d1) + abs(d3 - d2) );
				
				
				if tmp_err < err
					err = tmp_err;
					depth = d3;
				end
			end
			
			depthMap(v, u) = depth;
			errorMap(v, u) = err;
		end
	end
	
	disparityMap(:,:, 1) = depthMap;
	disparityMap(:,:, 2) = errorMap;
end
