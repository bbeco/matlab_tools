function [rImg1, rImg2] = rectifyImages(img1, img2, loc1, loc2, ...
		orient1, orient2)
	%RECTIFYIMAGES Rectify an image pair
	%   This function rectifies a pairs of equirectangular images so that their
	%   x-axis are aligned and their z-axis points in the same direction.
	%
	
	% Determining the rotation axis and angle for each camera to have the x-axis
	% aligned.
	t = loc2 - loc1;
	t = t/norm(t);
	x1 = [1, 0, 0] * orient1;
	n1 = cross(x1, t);
	n1 = n1/norm(n1);
	alpha1 = acos(dot(x1, t));
	rot1 = axisRot2mat(n1, alpha1);
	rImg1 = rotateLL(img1, rot1);
	
	x2 = [1, 0, 0] * orient2;
	n2 = cross(x2, t);
	n2 = n2/norm(n2);
	alpha2 = acos(dot(x2, t));
	rot2 = axisRot2mat(n2, alpha2);
	rImg2 = rotateLL(img2, rot2);
	
end

