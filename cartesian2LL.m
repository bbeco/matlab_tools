function [lat, long] = cartesian2LL(p)
%   Convert the given point to LL coordinates. The coordinate system of the
%   input is assumed to have x and y axes pointing right and up, and the -z
%   axe that points forward.
%   
%   Input:
%       -p: a 1x3 point vector
%   
%   Output:
%       -[lat, long]: 1x2 vector whose components are the latitude and 
%       longitude angles.
%

    long = arctan(-p(3), p(1));
    lat = arctan(sqrt(p(1)^2 + p(3)^2), p(2));
end

function theta = arctan(x, y)
    switch sign(x)
        case 0
            theta = sign(y)*pi/2;
        case 1
            if y ~= 0
                theta = atan(y/x);
            else
                theta = 0;
            end
        case -1
            if y>=0
                theta = atan(y/x) + pi;
            else
                theta = atan(y/x) - pi;
            end
    end
end