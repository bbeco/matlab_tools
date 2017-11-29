# 3D Environments Reconstruction Using 360° Videos

This repository contains all the source code I wrote for my master thesis, in particular, it contains the MATLAB scripts of my SfM and Dense Point Cloud Reconstruction Pipeline for equirectangular video sequences (produced with a Ricoh Theta S, a full-spherical panoramic camera).

The pipeline is composed of two steps: camera motion estimation and dense reconstruction. For the first step it estimates the essential matrix for each image pairs and decompose it to obtain the position of the camera in the second view according to the first view. The complete trajectory is the results of the composition of the local movement for each image pair. Drift is reduced with a windowed bundle adjustment.

The second phase creates a dense point cloud using disparity maps: the pipeline selects image pairs and computes the disparity maps with our adaptive block-matching algorithm.

For more details on how this pipeline works, see [my thesis](https://) for Markdown parsing

## Abstract
360° degrees or full spherical images are gaining a huge interest in different fields such as autonomous driving, cinematography, augmented reality (AR), and virtual reality (VR).

Computer vision research addressing spherical images is less popular than the one that considers traditional perspective cameras. This new kind of devices have some advantages with respect to standard cameras, for example, it allows users to capture an entire environment in a single shot.

In this work, we developed a structure from motion (SfM) pipeline for full spherical cameras composed of two main parts: camera poses estimation and dense point cloud reconstruction. This pipeline employs frames captured using a 360° video-camera in the equirectangular format.

Our contribution includes: a visual-based frame filter that selects frames to be used for motion estimation, a novel SfM pipeline implementation in MATLAB, and an adaptive window matching 
procedure for point cloud densification.

We tested the performance of our work both with synthetic 3D scenes and with real sequences captured with a Ricoh Theta S camera.

## Our Contribution
Our contribution includes:
* a simple but effective frame filter that selects the non-redundant frames to
be processed based on visual information only;
* a new approach to estimate poses that exploits both frontal and rear points;
* a novel block-matching algorithm for disparity map estimation for equirect-
angular images.
