%% Transform images
I1 = imread('images/image1.png');
I1 = rgb2gray(I1);
I2 = imread('images/image2.png');
I2 = rgb2gray(I2);
[I1up, I1down] = convertLL2Planar(I1);
imwrite(I1down, 'images/img1_planar_down.png');
[I2up, I2down] = convertLL2Planar(I2);
imwrite(I2down, 'images/img2_planar_down.png');

%% compute features
points1 = detectSURFFeatures(Iup);
imshow(Iup);
hold on;
for i = 1:size(points, 1)
	plot(points.Location(i, 1), points.Location(i, 2), 'g+');
end
hold off;