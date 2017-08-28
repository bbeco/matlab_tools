clear VARIABLES

load(fullfile('images', 'sfm_test', 'test4', 'groundTruth.mat'));

groundTruthPoses = alignOrientation(groundTruthPoses);

%% test 1
testNumber = 1;
estLocation = groundTruthPoses.Location;
estOrientation = groundTruthPoses.Orientation;

[locError, orientError, relLocError, relOrientError] = computePoseError(...
	estLocation, estOrientation, groundTruthPoses);

rightLocError = {[0 0 0]; [0 0 0]; [0 0 0]; [0 0 0]; [0 0 0]};
rightOrientError = {[0 0 0]; [0 0 0]; [0 0 0]; [0 0 0]; [0 0 0]};
rightRelLocError = {[0 0 0]; [0 0 0]; [0 0 0]; [0 0 0]; [0 0 0]};
rightRelOrientError = {[0 0 0]; [0 0 0]; [0 0 0]; [0 0 0]; [0 0 0]};

if isequal(locError, rightLocError) &&...
	isequal(orientError, rightOrientError) &&...
	isequal(relLocError, rightRelLocError) &&...
	isequal(relOrientError, rightRelOrientError)

	disp(['Test ', num2str(testNumber), ' OK']);
else
	error(['Test ', num2str(testNumber), ' FAILED']);
end

%% test 2
testNumber = 2;
ids = [1 3 5];
estLocation = groundTruthPoses.Location(ids, :);
estOrientation = groundTruthPoses.Orientation(ids, :);

[locError, orientError, relLocError, relOrientError] = computePoseError(...
	estLocation, estOrientation, groundTruthPoses, ids);

rightLocError = {[0 0 0]; [0 0 0]; [0 0 0]};
rightOrientError = {[0 0 0]; [0 0 0]; [0 0 0]};
rightRelLocError = {[0 0 0]; [0 0 0]; [0 0 0]};
rightRelOrientError = {[0 0 0]; [0 0 0]; [0 0 0]};

if isequal(locError, rightLocError) &&...
	isequal(orientError, rightOrientError) &&...
	isequal(relLocError, rightRelLocError) &&...
	isequal(relOrientError, rightRelOrientError)

	disp(['Test ', num2str(testNumber), ' OK']);
else
	error(['Test ', num2str(testNumber), ' FAILED']);
end

%% test 3
testNumber = 3;
ids = [1 3 5];
estLocation = groundTruthPoses.Location(ids, :);
estOrientation = groundTruthPoses.Orientation(ids, :);

for i = 1:length(ids)
	estLocation{i} = estLocation{i} + [1 0 0];
end

[locError, orientError, relLocError, relOrientError] = computePoseError(...
	estLocation, estOrientation, groundTruthPoses, ids);

rightLocError = {[-1 0 0]; [-1 0 0]; [-1 0 0]};
rightOrientError = {[0 0 0]; [0 0 0]; [0 0 0]};
rightRelLocError = {[0 0 0]; [0 0 0]; [0 0 0]};
rightRelOrientError = {[0 0 0]; [0 0 0]; [0 0 0]};

if isequal(locError, rightLocError) &&...
	isequal(orientError, rightOrientError) &&...
	isequal(relLocError, rightRelLocError) &&...
	isequal(relOrientError, rightRelOrientError)

	disp(['Test ', num2str(testNumber), ' OK']);
else
	error(['Test ', num2str(testNumber), ' FAILED']);
end
