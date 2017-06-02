function [x, y, z] = LL2Cartesian(lat, long)
    % compute the cartesian coordinates of the given point on the unitary 
    % sphere. The coordinate systems has axes that satisfy the right-hand 
    % rule; the x, y axes point right and up, and the negative z-axe points
    % forward.
    v = zeros(3, 1);
    p = cos(lat);
    v(1) = p*cos(long);
    v(2) = p*sin(long);
    v(3) = sin(lat);

    if nargout == 1
        x = v;
    elseif nargout == 3
        x = v(1);
        y = v(2);
        z = v(3);
    end
end