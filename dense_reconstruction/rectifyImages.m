function [rImg1, rImg2, rotation] = rectifyImages(img1, img2, loc1, loc2, ...
		orient1, orient2)
	%RECTIFYIMAGES Rectify an image pair
	%   This function rectifies a pairs of equirectangular images so that their
	%   x-axis are aligned and their z-axis points in the same direction. Then
	%   the image coordinates x and y directions are swapped to align
	%   coorrespondences on the meridian (vertical lines in the equirectangular
	%   mapping).
	%
	
	% Converting orientation matrixes to premultiply form (so that we don't get
	% confused by many different forms)
	orient1 = orient1';
	orient2 = orient2';
	
	% In order to rectify the image pair, we apply several rotations to the
	% cameras: rot1X align the x axis of the camera X to the epipole of the
	% other camera, rot22 align the z axis of camera 2 such that it becames
	% parallel to the other camera's, rot3 is a 90 degree rotation about the Z
	% axis.
	% See "3-D Reconstruction from Full-view Fisheye Camera" by Chuiwen et al.
	% for further information about image rectification
	
	% This rotation rotates the cameras on their z-axis so that the meridians
	% are the epipolar lines
	rot3 = eul2rotm([pi/2, 0, 0]);
	
	% Determining the rotation axis and angle for each camera to have the x-axis
	% aligned.
	t = loc2 - loc1;
	t = t/norm(t);
	x1 = orient1 * [1, 0, 0]';
	n1 = cross(x1, t);
	n1 = n1/norm(n1);
	alpha1 = acos(dot(x1, t));
	rot11 = axisRot2mat(orient1' * n1', alpha1);
	absrot11 = axisRot2mat(n1, alpha1);
	rotation = rot11 * rot3;
	rImg1 = rotateLL(img1, rotation);
	
	x2 = orient2 * [1, 0, 0]';
	n2 = cross(x2, t);
	n2 = n2/norm(n2);
	alpha2 = acos(dot(x2, t));
	rot12 = axisRot2mat(orient2' * n2', alpha2);
	absrot12 = axisRot2mat(n2, alpha2);
	% rotate second camera such that its z-axis is parallel to the other one's
	rot22 = ((absrot11 * orient1)' * (absrot12 * orient2))';
	
	rImg2 = rotateLL(img2, rot12 * rot22 * rot3);
	
end

