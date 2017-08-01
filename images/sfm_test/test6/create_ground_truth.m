viewId = [1, 2, 3, 4, 5]';
Location = {
% Locations
% Camera
[ 0.0 0.0 0.0]
% Camera.001
[ -1.9539607763290405 -0.1302490383386612 1.6393083333969116]
% Camera.002
[ -3.289252519607544 0.6095656752586365 3.7146596908569336]
% Camera.003
[ -4.396525859832764 1.5942485332489014 4.16454553604126]
% Camera.004
[ -5.424428939819336 4.126290798187256 5.564675807952881]
};

Orientation = {
% Orientations
% Camera
[ 1.0 0.0 0.0; 0.0 7.549790126404332e-08 -1.0; 0.0 1.0 7.549790126404332e-08 ]
% Camera.001
[ 0.9878800511360168 0.0269535593688488 -0.15286117792129517; -0.15521931648254395 0.17154361307621002 -0.9728718996047974; 0.0 0.9848077297210693 0.17364822328090668 ]
% Camera.002
[ 0.9914641380310059 0.056729141622781754 -0.11739125847816467; -0.1303798407316208 0.4313926696777344 -0.8926934003829956; 0.0 0.9003788828849792 0.43510669469833374 ]
% Camera.003
[ 0.9497714638710022 0.10066678375005722 -0.2963111698627472; -0.3129442632198334 0.30551907420158386 -0.8992908000946045; 0.0 0.9468496441841125 0.3216764032840729 ]
% Camera.004
[ 0.7604511380195618 0.5212012529373169 -0.38738012313842773; -0.6493951678276062 0.6103342175483704 -0.45362770557403564; 0.0 0.5965244770050049 0.8025949001312256 ]
	};
groundTruthPoses = table(viewId, Location, Orientation);

save('images/sfm_test/test6/groundTruth.mat', 'groundTruthPoses');
