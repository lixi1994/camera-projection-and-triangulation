# Camera-Projection-and-Triangulation
Project 2 of CMPEN454 Computer Vision

The goal of this project is to implement forward (3D point to 2D point) and inverse (2D point to 3D ray) camera projection, and to perform triangulation from two cameras to do 3D reconstruction from pairs of matching 2D image points.

Input:

  Camera calibration parameters (Intrinsic and extrinsic) for two video cameras:
    vue2CalibInfo.mat
    vue4CalibInfo.mat
  3D point data for each of 12 body joints:
    Subject4-Session3-Take4_mocapJoints.mat
  mp4 movie files containing the video frames recorded by each of the two video cameras:
    Subject4-Session3-24form-Full-Take4-Vue2.mp4
    Subject4-Session3-24form-Full-Take4-Vue4.mp4
    
Output:
  Projecting 3D points into 2D pixel locations:
    2D_projection_vue2.jpg
    2D_projection_vue4.jpg
  Triangulation back into a set of 3D scene points:
    3Dpoints.mp4
 
Run main_project2.m to do camera projection and triangulation.
Run plotmovie.m to make a movie comparing original and reconstructed skeletons.
