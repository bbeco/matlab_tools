clear 
addpath('utils');
addpath('dense_reconstruction');
addpath('coordinate_transform');
img = rgb2gray(...
	imread(fullfile('images/densification_test/test1/rec01_gt.png')));
[height, width] = size(img);
patch = createPatch(img, 0, 0, width, height);
figure
subplot(2, 1, 1);
imshow(img);
ax = subplot(2, 1, 2);
imshow(patch{1});

while true
	[x, y, button] = ginput(1);
	if button == 27
		break;
	end
	round([x, y])
	[lat, long] = extractLLCoordinateFromImage(x, y, width, height);
	patch = createPatch(img, lat, long, width, height);
	imshow(patch{1}, 'Parent', ax);
end