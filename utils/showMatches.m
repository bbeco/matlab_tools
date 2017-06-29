function showMatches(I1, I2, points1, points2, selectedPoint)
	subplot(2, 1, 1);
	I1 = insertMarker(I1, points1, 'o', 'Color', 'red');
	imshow(I1);
	
	subplot(2, 1, 2);
	imshow(I2);
	
	if selectedPoint(1) < 0 || selectedPoint(2) < 0
		return;
	end

	i = closestPoint(selectedPoint, points1.Location);

	subplot(2, 1, 1);
	hold on;
	x = points1.Location(i, :);
	plot(x(1), x(2),'g+');
	hold off;

	subplot(2, 1, 2);
	x = points2.Location(i, :);
	hold on;
	plot(x(1), x(2), 'go');
	hold off;
	
end