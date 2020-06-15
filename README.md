# las2pcd
(c) by Arnadi Murtiyoso

Photogrammetry and Geomatics Group, ICube UMR 7357 INSA Strasbourg

Simple function written in C++ used to convert .LAS point cloud to PCL-compatible .PCD format.

Required dependencies:
- PCL (http://pointclouds.org/)
- libLAS (https://www.liblas.org/)

Ubuntu installation:
	
	apt-get install -y git cmake
	
	apt-get install -y libpcl-dev liblas-dev liblas-c-dev
	
	git clone -b ubuntu-singularity https://github.com/gearslaboratory/las2pcd.git
	
	cd las2pcd
	
	cmake .
	
	make
	
	mv las2pcd /usr/bin/

Change log:

v0.1 (24 May 2017):
- created simple function to convert .las files to .pcd files

v0.2 (29 May 2017):
- added code to import .las point cloud color
- resolved problem with color scale difference between .las and PCL

v0.3 (16 March 2018), Jonathan Greenberg's fork (jgreenberg@unr.edu)
- cmake tweaked to work on standard Ubuntu systems (designed for a singularity install)
- requires:
apt-get install -y libpcl-dev liblas-dev liblas-c-dev

v0.4 (15 June 2020):
- added code to retrieve intensity value from the .las file (credit to Antoine Carreaud)
