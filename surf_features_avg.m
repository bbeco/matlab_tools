% This script counts the average number of SURF points found in a set of 
% images

% Where to find the images
path = '/home/andrea/syntetic_scene';
% images name format (ex. 0123.png)
expression = '\d{4}\.png';
files = dir(path);
cd(path);
found_features = zeros(length(files), 1);
j = 0;
for i = 1:length(files)
    match = regexp(files(i).name, expression);
    if (~isempty(match))
        I = rgb2gray(imread(files(i).name));
        %I = imread(files(i).name);
        points = detectSURFFeatures(I);
        j = j + 1;
        found_features(j) = points.Count;
    end
end
found_features = found_features(1:j);
%% display results
fprintf('number of images: %d\nAverage number of features detected per image: %f\n25th-percentile: %d\n75th-percentile: %d\n', j, mean(found_features), prctile(found_features, 25), prctile(found_features, 75));
