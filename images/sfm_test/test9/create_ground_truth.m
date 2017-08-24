ViewId = [1, 2, 3, 4, 5]';
% Location
Location = {
% Camera
[ 0.0 -0.0 0.0 ]
% Camera.001
[ 15.0 -0.0 0.05647033452987671 ]
% Camera.002
[ 30.0 -0.0 8.0 ]
% Camera.003
[ 40.0 -0.0 28.0 ]
% Camera.004
[ 41.28864288330078 -8.940696716308594e-07 42.7489128112793 ]
};
% Orientation
Orientation = {
% Camera
[ 1.0 0.0 0.0;
 0.0 7.549790126404332e-08 1.0;
 0.0 -1.0 7.549790126404332e-08 ]
% Camera.001
[ 0.9396926164627075 -0.34202006459236145 0.0;
 2.5821796256764173e-08 7.094482157299353e-08 1.0;
 -0.34202006459236145 -0.9396926164627075 7.549790126404332e-08 ]
% Camera.002
[ 0.9063078165054321 -0.42261818051338196 0.0;
 3.1906786546187504e-08 6.84243346427138e-08 1.0;
 -0.42261818051338196 -0.9063078165054321 7.549790126404332e-08 ]
% Camera.003
[ 0.6427875757217407 -0.7660444974899292 0.0;
 5.783475032217211e-08 4.8529113172435245e-08 1.0;
 -0.7660444974899292 -0.6427875757217407 7.549790126404332e-08 ]
% Camera.004
[ 0.42261824011802673 -0.9063078165054321 0.0;
 6.84243346427138e-08 3.190679009890118e-08 1.0;
 -0.9063078165054321 -0.42261824011802673 7.549790126404332e-08 ]
};
groundTruthPoses = table(ViewId, Orientation, Location);

save('images/sfm_test/test9/groundTruth.mat', 'groundTruthPoses');
