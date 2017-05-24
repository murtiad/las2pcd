#include <iostream>
#include <cstdlib>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <liblas/liblas.hpp>

using namespace std;

int main (int argc, char** argv)
{
	string filePath;

	cout << "Enter full file (.las) path: " << endl;
    
	getline(cin, filePath);

    std::cerr << "INFO : Loading : " << filePath << std::endl;
    
    // creating a new PCL pointcloud
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Opening  the las file
    std::ifstream ifs(filePath.c_str(), std::ios::in | std::ios::binary);
    if(ifs.fail()) {
        std::cerr << "ERROR : Impossible to open the file : " << filePath <<std::endl;
        return 1;
    }

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs); // reading las file
    unsigned long int nbPoints=reader.GetHeader().GetPointRecordsCount();

	// Fill in the cloud data
	cloud.width    = nbPoints;
	cloud.height   = 1;
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);

	std::cout << cloud.points.size () <<std::endl;
	int i=1;

	while(reader.ReadNextPoint()) 
	{
		cloud.points[i].x = (reader.GetPoint().GetX());
	    cloud.points[i].y = (reader.GetPoint().GetY());
	    cloud.points[i].z = (reader.GetPoint().GetZ());
		i++;
	}
  
	pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  
	std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;
	
	return (0);
}