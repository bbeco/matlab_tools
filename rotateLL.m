function rImg = rotateLL(img, x, y, z)
    %Compute a rotation of the image in lat/long format.
    [height, width] = size(img);
    rImg = zeros(height, width, 'uint8');
    for i = 1:height
        for j = 1:width
            rlat = pi/2 - i*pi/height;
            rlong = j*2*pi/width - pi;
            p = LL2Cartesian(rlat,rlong);
            %rotation
            Rx = [1 0 0; 0 cos(x) -sin(x); 0 sin(x) cos(x)];
            Ry = [cos(y) 0 sin(y); 0 1 0; -sin(y) 0 cos(y)];
            Rz = [cos(z) -sin(z) 0; sin(z) cos(z) 0; 0 0 1];
            p = Rz*Ry*Rx*p;
            [lat, long] = cartesian2LL(p);
            %mapping
            u = (long + pi)/(2*pi)*width;
            v = (pi/2 - lat)/pi*height;
            rImg(i, j) = img(max(1, ceil(v)), max(1, ceil(u)));
        end
    end
end