I = imread('../rotation_test/original.png');
Gx = imread('../rotation_test/ground_truth.png');
Gy = imread('../rotation_test/ground_truth2.png');
Gz = imread('../rotation_test/ground_truth3.png');

xRot = rotateLL(I, -pi/2, 0, 0);
yRot = rotateLL(I, 0, -pi/2, 0);
zRot = rotateLL(I, 0, 0, -pi/2);

%% display
%x rotation
subplot(3, 3, [1 2 3]);
imshow(I);
title('Original');

subplot(3, 3, 4);
imshow(xRot);
title('Rotation around X-axis');

subplot(3, 3, 7);
imshow(Gx);
title('Ground truth');

%y rotation
subplot(3, 3, 5);
imshow(yRot);
title('Rotation around Y-axis');

subplot(3, 3, 8);
imshow(Gy);
title('Ground truth');

%z rotation
subplot(3, 3, 6);
imshow(zRot);
title('Rotation around Z-axis');

subplot(3, 3, 9);
imshow(Gz);
title('Ground truth');