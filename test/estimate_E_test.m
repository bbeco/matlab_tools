addpath(genpath('..'));
imageDir = fullfile('images', 'essential_matrix_test', {'ll0.png', 'll1.png', 'll2.png', 'll3.png', 'll4.png', 'll5.png', 'll6.png'});

imds = imageDatastore(imageDir);
% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
    I = readimage(imds, i);
    images{i} = rgb2gray(I);
end
[orientation, translation, algebraicError, validPointsFraction] = computeRotoTranslation(images{1}, images{2})