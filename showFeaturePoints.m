function showFeaturePoints(img, points)
	imshow(img);
	n = size(points.Location, 1);
	hold on;
	for i = 1:n
		plot(points.Location(i, 1), points.Location(i, 2), 'g+');
	end
	hold off;