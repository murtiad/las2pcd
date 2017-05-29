# las2pcd
(c) by Arnadi Murtiyoso
Photogrammetry and Geomatics Group, ICube UMR 7357 INSA Strasbourg

Simple function written in C++ used to convert .LAS point cloud to PCL-compatible .PCD format.

Required dependencies:
- PCL (http://pointclouds.org/)
- libLAS (https://www.liblas.org/)

Change log:
v0.1 (24 May 2017):
- created simple function to convert .las files to .pcd files

v0.2 (29 May 2017:
- added code to import .las point cloud color
- resolved problem with color scale difference between .las and PCL
