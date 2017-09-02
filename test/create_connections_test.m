%% Init
clear VARIABLES;
addpath(fullfile('..'));
f1 = rand(1, 64);
f2 = rand(1, 64);
f3 = rand(1, 64);
f4 = rand(1, 64);

%% ************ TEST1 *************
testId = 1;
vSet = viewSet;
vWindow = ViewWindow(2);

% View 1
vSet = addView(vSet, 1, 'Points', [1 1; 4 4], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 1, [1 1; 4 4], [f1; f2], [1 1; 4 4]);

% View 2
vSet = addView(vSet, 2, 'Points', [2 2; 5 5], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 2, [2 2; 5 5], [f2; f1], [2 2; 5 5]);
addConnection(vWindow, 1, 2, [1, 2; 2, 1]);
vSet = computeTrackAndCreateConnections(vSet, vWindow);

% View 3
vSet = addView(vSet, 3, 'Points', [3 3], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 3, [3 3], f1, [3 3]);
addConnection(vWindow, 2, 3, [2, 1]);
vSet = computeTrackAndCreateConnections(vSet, vWindow);

% View 4
vSet = addView(vSet, 4, 'Points', [6 6], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 4, [6 6], f1, [6 6]);
addConnection(vWindow, 3, 4, [1, 1]);
vSet = computeTrackAndCreateConnections(vSet, vWindow);

if isequal(cat(1,vSet.Connections.Matches{:}), [1 2; 2 1; 2 1; 1 1])
    disp(['Test ', num2str(testId), ' OK']);
else
    error(['Test ', num2str(testId), ' FAILED']);
end

%% ************ TEST2 *************
testId = 2;
vSet = viewSet;
vWindow = ViewWindow(3);

% View 1
vSet = addView(vSet, 1, 'Points', [1 1; 4 4], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 1, [1 1; 4 4], [f1; f2], [1 1; 4 4]);

% View 2
vSet = addView(vSet, 2, 'Points', [2 2; 5 5], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 2, [2 2; 5 5], [f2; f1], [2 2; 5 5]);
addConnection(vWindow, 1, 2, [1, 2; 2, 1]);

% View 3
vSet = addView(vSet, 3, 'Points', [3 3], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 3, [3 3], f1, [3 3]);
addConnection(vWindow, 2, 3, [2, 1]);
vSet = computeTrackAndCreateConnections(vSet, vWindow);

% View 4
vSet = addView(vSet, 4, 'Points', [6 6], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 4, [6 6], f1, [6 6]);
addConnection(vWindow, 3, 4, [1 1]);
vSet = computeTrackAndCreateConnections(vSet, vWindow);

if isequal(cat(1,vSet.Connections.Matches{:}), [1 2; 1 1; 2 1; 2 1; 1 1])
    disp(['Test ', num2str(testId), ' OK']);
else
    error(['Test ', num2str(testId), ' FAILED']);
end

%% ************ TEST3 *************
testId = 3;
vSet = viewSet;
vWindow = ViewWindow(4);

% View 1
vSet = addView(vSet, 1, 'Points', [1 1; 4 4], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 1, [1 1; 4 4], [f1; f2], [1 1; 4 4]);

% View 2
vSet = addView(vSet, 2, 'Points', [2 2; 5 5], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 2, [2 2; 5 5], [f2; f1], [2 2; 5 5]);
addConnection(vWindow, 1, 2, [1, 2; 2, 1]);

% View 3
vSet = addView(vSet, 3, 'Points', [3 3], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 3, [3 3], f1, [3 3]);
addConnection(vWindow, 2, 3, [2 1]);

% View 4
vSet = addView(vSet, 4, 'Points', [6 6], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 4, [6 6], f2, [6 6]);
addConnection(vWindow, 3, 4, []);
vSet = computeTrackAndCreateConnections(vSet, vWindow);

if isempty(vSet.Connections)
    disp(['Test ', num2str(testId), ' OK']);
else
    error(['Test ', num2str(testId), ' FAILED']);
end

%% ************ TEST4 *************
testId = 4;
vSet = viewSet;
vWindow = ViewWindow(4);

% View 1
vSet = addView(vSet, 1, 'Points', [1 1; 4 4], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 1, [1 1; 4 4], [f1; f2], [1 1; 4 4]);

% View 2
vSet = addView(vSet, 2, 'Points', [2 2; 5 5], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 2, [2 2; 5 5], [f2; f1], [2 2; 5 5]);
addConnection(vWindow, 1, 2, [1, 2; 2, 1]);

% View 3
vSet = addView(vSet, 3, 'Points', [3 3], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 3, [3 3], f2, [3 3]);
addConnection(vWindow, 2, 3, [1, 1]);

% View 4
vSet = addView(vSet, 4, 'Points', [6 6], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 4, [6 6], f2, [6 6]);
addConnection(vWindow, 3, 4, [1 1]);
vSet = computeTrackAndCreateConnections(vSet, vWindow);

if isequal(cat(1, vSet.Connections.Matches{:}), [2 1; 2 1; 2 1; 1 1; 1 1; 1 1])
    disp(['Test ', num2str(testId), ' OK']);
else
    error(['Test ', num2str(testId), ' FAILED']);
end

%% ************ TEST5 *************
testId = 5;
vSet = viewSet;
vWindow = ViewWindow(4);

% View 1
vSet = addView(vSet, 1, 'Points', [1 1; 4 4], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 1, [1 1; 4 4], [f1; f2], [1 1; 4 4]);

% View 2
vSet = addView(vSet, 2, 'Points', [2 2; 5 5], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 2, [2 2; 5 5], [f3; f1], [2 2; 5 5]);
addConnection(vWindow, 1, 2, [1, 2]);

% View 3
vSet = addView(vSet, 3, 'Points', [3 3], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 3, [3 3], f2, [3 3]);
addConnection(vWindow, 2, 3, []);

% View 4
vSet = addView(vSet, 4, 'Points', [6 6], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 4, [6 6], f2, [6 6]);
addConnection(vWindow, 3, 4, [1 1]);
vSet = computeTrackAndCreateConnections(vSet, vWindow);

if isempty(vSet.Connections)
    disp(['Test ', num2str(testId), ' OK']);
else
    error(['Test ', num2str(testId), ' FAILED']);
end

%% ************ TEST6 *************
testId = 6;
vSet = viewSet;
vWindow = ViewWindow(4);

% View 1
vSet = addView(vSet, 1, 'Points', [1 1], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 1, [1 1], f1, [1 1]);

% View 2
vSet = addView(vSet, 2, 'Points', [2 2; 4 4], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 2, [2 2; 4 4], [f1; f2], [2 2; 4 4]);
addConnection(vWindow, 1, 2, [1 1]);

% View 3
vSet = addView(vSet, 3, 'Points', [3 3; 5 5], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 3, [3 3; 5 5], [f1; f2], [3 3; 5 5]);
addConnection(vWindow, 2, 3, [1, 1; 2, 2]);

% View 4
vSet = addView(vSet, 4, 'Points', [6 6; 7 7], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 4, [6 6; 7 7], [f1; f2], [6 6; 7 7]);
addConnection(vWindow, 3, 4, [1, 1; 2, 2]);
vSet = computeTrackAndCreateConnections(vSet, vWindow);

if isequal(cat(1, vSet.Connections.Matches{:}), ones(6, 2))
    disp(['Test ', num2str(testId), ' OK']);
else
    error(['Test ', num2str(testId), ' FAILED']);
end

%% ************ TEST7 *************
testId = 7;
vSet = viewSet;
vWindow = ViewWindow(4);

% View 1
vSet = addView(vSet, 1, 'Points', [1 1], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 1, [1 1], f1, [1, 1]);

% View 2
vSet = addView(vSet, 2, 'Points', [2 2; 4 4], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 2, [2 2], f1, [2 2]);
addConnection(vWindow, 1, 2, [1, 1]);

% View 3
vSet = addView(vSet, 3, 'Points', [3 3; 5 5], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 3, [3 3], f2, [5 5]);
% The following match is wrong but this is exactly what this case is designed
% for. This is a match that does not satisfy the track consistency check
addConnection(vWindow, 2, 3, [1, 1]);

% View 4
vSet = addView(vSet, 4, 'Points', [6 6; 7 7], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 4, [6 6], f2, [6 6]);
addConnection(vWindow, 3, 4, [1, 1]);
vSet = computeTrackAndCreateConnections(vSet, vWindow);

if isempty(vSet.Connections)
    disp(['Test ', num2str(testId), ' OK']);
else
    error(['Test ', num2str(testId), ' FAILED']);
end