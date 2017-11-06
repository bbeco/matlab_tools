function [disparityMap, dm_maxDisparity, patchesL, patchesL_sq, patchesL_dx, patchesR, patchesR_sq, patchesR_dx] = computeDisparityNCCEquirectangular(imgL, imgR, dm_patchSize, dm_maxDisparity, dm_horDisparity, dm_regularization, dm_alpha, dm_subtractMeanValue, patchesL, patchesL_sq, patchesL_dx, patchesR, patchesR_sq, patchesR_dx)
%COMPUTEDISPARITYEQUIRECTANGULAR Compute disparity map for an LL image pair
%   This function is inspired by computeDisparitySlow of HDR Toolbox.
	if(~exist('dm_patchSize', 'var'))
		dm_patchSize = 9;
	end

	if(~exist('dm_maxDisparity', 'var'))
		dm_maxDisparity = -1;
	end
	
	if ~exist('dm_horDisparity', 'var')
		dm_horDisparity = 0;
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
	
	if ~exist('dm_subtractMeanValue', 'var')
		dm_subtractMeanValue = true;
	end
	
	[r, c, col] = size(imgL);

	min_u = dm_patchSize + 1;
	max_u = c - dm_patchSize - 1;
	
	if col > 1
		imgL = rbg2gray(imgL);
		imgR = rgb2gray(imgR);
	end
	
    dm_alpha_inv = (1.0 - dm_alpha);
	
	disparityMap = zeros(r, c, 2);
	%temporary variables to use parfor
	depthMap = zeros(r, c);
	corrMap = zeros(r, c);
	
	% if the patches were not previously computed, do it now
	if ~exist('patchesL', 'var')
		patchesL = cell(r, c);
		patchesL_sq = cell(r, c);
		patchesL_dx = cell(r, c);
		patchesR = cell(r, c);
		patchesR_sq = cell(r, c);
		patchesR_dx = cell(r, c);
		parfor j = 1:c
			disp(['Creating patch: ', num2str(j), '/', num2str(c)]);
			for i = 1:r
				[lat, long] = extractLLCoordinateFromImage(j, i, c, r);
				[patchesL{i, j}, patchesL_sq{i, j}, patchesL_dx{i, j}] = createPatch(imgL, ...
					lat, long, c, r, dm_patchSize, dm_subtractMeanValue);
				[patchesR{i, j}, patchesR_sq{i, j}, patchesR_dx{i, j}] = createPatch(imgR, ...
					lat, long, c, r, dm_patchSize, dm_subtractMeanValue);
			end
		end
	end
    
	parfor u = min_u:max_u
		disp(['Processing column: ', num2str(u), '/', num2str(c)]);
		for v = (dm_patchSize + 1):(r - dm_patchSize - 1)
			%removed to use parfor
% 			d1 = disparityMap(v - 1, u, 1);
% 			d2 = disparityMap(v, u - 1, 1);
			
			min_v = max([v - dm_maxDisparity, dm_patchSize + 1]);
			max_v = min([v + dm_maxDisparity, r - dm_patchSize - 1]);
			
            % if we use the sum of the SSD response, decomment this
            % otherwise leave it as it is.
			%lambda = dm_regularization / (max_v - min_v + 1);
%             lambda = dm_regularization;
			corr = -1;
			depth = 0;
			
			min_l = max([u - dm_horDisparity, 1]);
			max_l = min([u + dm_horDisparity, c]);
			
			for k = min_v:max_v
				for l = min_l:max_l
					%NCC
					delta = patchesL{v, l}.*patchesR{k, l};
					den = sqrt(sum(patchesL_sq{v, l}(:)) * sum(patchesR_sq{k, l}(:)));

					% gradient matching
					delta_dx_sq = (patchesL_dx{v, l} - patchesR_dx{k, l}).^2;

					tmp_corr = dm_alpha_inv * sum(delta(:))/den - dm_alpha * sum(delta_dx_sq(:));
					% simplified formula
	%  				tmp_err = sum(delta(:));
					d3 = k - v;

					% removed just like regularization
					%tmp_err = tmp_err + lambda * (abs(d3) + abs(d3 - d1) + abs(d3 - d2) );


					if tmp_corr > corr
						corr = tmp_corr;
						depth = d3;
					end
				end
			end
			
			depthMap(v, u) = depth;
			corrMap(v, u) = corr;
		end
	end
	
	disparityMap(:,:, 1) = depthMap;
	disparityMap(:,:, 2) = corrMap;
end

