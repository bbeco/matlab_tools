function [x, y, z] = LL2Cartesian(lat, long)
%     compute the cartesian coordinates of the given point on the unitary 
%     sphere. The coordinate systems has axes that satisfy the right-hand 
%     rule; the x, y axes point right and up, and the negative z-axe points
%     forward.
% 
% 	Input:
% 		-lat: latitude value (in radians)
% 		-long: longitude value (in radians)
% 		
% 	Output:
% 		-[x, y, z]: an 1x3 vector with the resulting cartesian coordinates
% 		
		
    v = zeros(3, 1);
    p = cos(lat);
    v(1) = p*sin(long);
    v(2) = sin(lat);
    v(3) = -p*cos(long);

    if nargout == 1
        x = v;
    elseif nargout == 3
        x = v(1);
        y = v(2);
        z = v(3);
    end
end