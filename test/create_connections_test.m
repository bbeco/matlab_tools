clear VARIABLES;
addpath(fullfile('..'));
vSet = viewSet;
vWindow = ViewWindow(4);
f1 = rand(1, 64);
f2 = rand(1, 64);
f3 = rand(1, 64);
f4 = rand(1, 64);

% View 1
vSet = addView(vSet, 1, 'Points', [1 1; 4 4], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 1, [1 1; 4 4], [f1; f2], [1 1; 4 4]);

% View 2
vSet = addView(vSet, 2, 'Points', [2 2; 5 5], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 2, [2 2; 5 5], [f2; f1], [2 2; 5 5]);
% vSet = computeTrackAndCreateConnections(vSet, vWindow, [1, 2; 2, 1]);

% View 3
vSet = addView(vSet, 3, 'Points', [3 3], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 3, [3 3], f1, [3 3]);
% vSet = computeTrackAndCreateConnections(vSet, vWindow, [2, 1]);

% View 4
vSet = addView(vSet, 4, 'Points', [6 6], 'Orientation', eye(3), 'Location', [0 0 0]);
addPoints(vWindow, 4, [6 6], f1, [6 6]);
vSet = computeTrackAndCreateConnections(vSet, vWindow, [1, 1]);

% if isequal(cat(1,vSet.Connections.Matches{:}), [1 2; 2 1; 1 1; 1 1])
%     disp('Test 1 OK');
% else
%     error('Test 1 FAILED');
% end
