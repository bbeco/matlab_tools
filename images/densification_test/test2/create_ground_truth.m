ViewId = [1 2 3]';
% Location
Location = {
% camera1
[ -2.5776872634887695 -2.4782044887542725 0.0 ]
% camera2
[ -0.5776900053024292 -0.4781999886035919 0.0 ]
% camera3
[ -0.5776900053024292 -2.4781999588012695 2.0 ]
};
% Orientation
Orientation = {
% camera1
[ 1.0 0.0 0.0;
 0.0 7.549790126404332e-08 -1.0;
 0.0 1.0 7.549790126404332e-08 ]
% camera2
[ 0.8660253286361694 0.5000001192092896 0.0;
 -3.77489612901627e-08 6.538309804682285e-08 -1.0;
 -0.5000001192092896 0.8660253286361694 7.549790126404332e-08 ]
% camera3
[ 1.0 0.0 0.0;
 0.0 0.4999999701976776 -0.866025447845459;
 0.0 0.866025447845459 0.4999999701976776 ]
};
originalPoses = table(ViewId, Orientation, Location);

ViewId = [1, 2]';
Location = {
% rectCam1
[ -2.5776872634887695 -2.4782044887542725 0.0 ]
% rectCam2
[ -0.5776900053024292 -0.4781999886035919 0.0 ]
};
Orientation = {
% rectCam1
[ 7.549790126404332e-08 0.0 -1.0;
 -0.7071065902709961 -0.7071069478988647 -5.3385065257316455e-08;
 -0.7071069478988647 0.7071065902709961 -5.338509012631221e-08 ]
% rectCam2
[ 7.549790126404332e-08 0.0 -1.0;
 -0.7071065902709961 -0.7071069478988647 -5.3385065257316455e-08;
 -0.7071069478988647 0.7071065902709961 -5.338509012631221e-08 ]
};
rectifiedPosesGT = table(ViewId, Orientation, Location);

save('images/densification_test/test2/poses.mat', ...
	'originalPoses', 'rectifiedPosesGT');