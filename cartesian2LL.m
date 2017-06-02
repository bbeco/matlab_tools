function [lat, long] = cartesian2LL(p)
    long = arctan(p(1), p(2));
    lat = arctan(sqrt(p(1)^2 + p(2)^2), p(3));
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