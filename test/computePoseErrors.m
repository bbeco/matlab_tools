clear VARIABLES;
addpath(fullfile('utils'));
load(fullfile('images', 'sfm_test', 'test4', 'groundTruth.mat'));
%% Test 1
estLocation = groundTruthPoses.Location;
estOrientation = groundTruthPoses.Orientation;

[locError, orientError] = computePoseError(estLocation, estOrientation, ...
	groundTruthPoses);

locError
orientError

%% Test 2
clear estLocation;
clear estOrientation;
estLocation{1, 1} = groundTruthPoses.Location{4};
estLocation{2, 1} = groundTruthPoses.Location{5};
estOrientation{1, 1} = groundTruthPoses.Orientation{4};
estOrientation{2, 1} = groundTruthPoses.Orientation{5};

[locError, orientError] = computePoseError(estLocation, estOrientation, ...
	groundTruthPoses, [4, 5]);

locError
orientError

%% Test 3
clear estLocation;
clear estOrientation;
estLocation{1, 1} = groundTruthPoses.Location{1};
estLocation{2, 1} = groundTruthPoses.Location{2};
estOrientation{1, 1} = groundTruthPoses.Orientation{1};
estOrientation{2, 1} = groundTruthPoses.Orientation{2};

[locError, orientError] = computePoseError(estLocation, estOrientation, ...
	groundTruthPoses, [4, 5]);

locError
orientError