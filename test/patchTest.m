clear 
addpath('utils');
addpath('dense_reconstruction');
addpath('coordinate_transform');
addpath('geometry');
img = imread(fullfile('images/densification_test/test3/rImg1.jpg'));
[height, width, channels] = size(img);
if channels > 1
	img = rgb2gray(img);
end
patch = createPatch(img, 0, 0, width, height);
figure
subplot(2, 1, 1);
imshow(img);
ax = subplot(2, 1, 2);
imshow(uint8(255 * mat2gray(patch{1})));

while true
	[x, y, button] = ginput(1);
	if button == 27
		break;
	end
	round([x, y]);
	[lat, long] = extractLLCoordinateFromImage(x, y, width, height);
	patch = createPatch(img, lat, long, width, height);
	imshow(uint8(255 * mat2gray(patch{1})), 'Parent', ax);
end