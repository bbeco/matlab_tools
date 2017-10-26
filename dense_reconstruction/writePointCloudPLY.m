function writePointCloudPLY(points, colors, name)
%
%
%       writePointCloudPLY(points, colors, name)
%
%
% Digit
% An automatic MATLAB app for the digitalization of archaeological drawings. 
% http://vcg.isti.cnr.it
% 
% Copyright (C) 2016-17
% Visual Computing Laboratory - ISTI CNR
% http://vcg.isti.cnr.it
% Main author: Francesco Banterle
% 
% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at http://mozilla.org/MPL/2.0/.
%

fid = fopen(name, 'wt');

size_points = size(points, 1);    

fprintf(fid, 'ply \n');
fprintf(fid, 'format ascii 1.0 \n');
fprintf(fid, 'element vertex %d \n' , size_points);
fprintf(fid, 'property float x \n');
fprintf(fid, 'property float y \n');
fprintf(fid, 'property float z \n');

fprintf(fid, 'property uchar red \n');
fprintf(fid, 'property uchar green \n');
fprintf(fid, 'property uchar blue \n');
fprintf(fid, 'property uchar alpha \n');

fprintf(fid, 'end_header \n');


for i=1:size_points
      
    fprintf(fid, '%.3f %.3f %.3f' , points(i,1), points(i,2), points(i,3));         
      
    fprintf(fid, ' %d %d %d 255\n' , colors(i,1), colors(i,2), colors(i,3));         
end    

fclose(fid);  

end