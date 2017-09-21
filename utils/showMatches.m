function showMatches(I1, I2, points1, points2, selectedPoint)
	
	if ~isnumeric(points1)
		points1 = points1.Location;
	end
	
	if ~isnumeric(points2)
		points2 = points2.Location;
	end
	
	subplot(2, 1, 1);
% 	I1 = insertMarker(I1, points1, 'o', 'Color', 'red', 'size', 1000);
	imshow(I1);
	hold on
	plot(points1(:, 1), points1(:, 2), 'ro', 'MarkerSize', 10);
	hold off
	
	subplot(2, 1, 2);
	imshow(I2);
	
	if selectedPoint(1) < 0 || selectedPoint(2) < 0
		return;
	end

	i = closestPoint(selectedPoint, points1);

	subplot(2, 1, 1);
	hold on;
	x = points1(i, :);
	plot(x(1), x(2),'g+', 'MarkerSize', 10);
	hold off;

	subplot(2, 1, 2);
	x = points2(i, :);
	hold on;
	plot(x(1), x(2), 'go', 'MarkerSize', 10);
	hold off;
	
end