#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <cstdlib>
#include <iostream>
#include "getopt.h"

struct Settings
{
	std::string inputFile;  // Input filename
	std::string outputFile; // Output filename
	bool mesh;              // Save mesh
	double searchRadius;	// Search radius, default 0.025
	double maxDistance;		// Distance between points, default 2.5
	int maxNeighbors;		// Neighbors searched, default 100
	double minAngle;		// Min angle of triangles, not guranteed, default 10
	double maxAngle;		// Max angle of triangles, default 120
	bool normalConsistency; // Use normal consistency, default false
	double maxSurfaceAngle; // Used with above for smoothing, default 45
	bool verbose;

	Settings() : mesh(false), searchRadius(0.025), maxDistance(2.5), maxNeighbors(100),
		minAngle(M_PI/18), maxAngle(2*M_PI/3), maxSurfaceAngle(M_PI/4), normalConsistency(false),
		verbose(false)
		{ inputFile = "\0"; outputFile = "\0"; }
};

void printHelp()
{
	std::cout << "Basic usage: converter <input pcd file> <output file>\n"
		<< "\t-m, --mesh\t If set will save a mesh instead of point cloud\n"
		<< "\t-s, --search-radius\t Search radius used to collect points,\n\t\t\tdefault 0.025\n"
		<< "\t-d, --max-distance\t Set max distance for points to be accepted,\n\t\t\tdefault 2.5\n"
		<< "\t-n, --max-neighbors\t Maximum Nearest Neighbors searched,\n\t\t\tdefault 100\n"
		<< "\t-u, --minimum-angle\t Minimum angle of triangles, not guranteed,\n\t\t\tdefault 10 degrees\n"
		<< "\t-x, --maximum-angle\t Maximum angle of triangles,\n\t\t\tdefault 120 degrees\n"
		<< "\t-c, --normal-consistency If normal consistency is set,\n\t\t\tdefault false\n"
		<< "\t-a, --surface-angle\t Max surface angle for normal consistency,\n\t\t\tdefault 45\n"
		<< "\t--verbose\t Set verbosity\n";
}

void saveMesh(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, Settings s)
/* Greedy Projection from Point Cloud to mesh */
{
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    gp3.setSearchRadius(s.searchRadius);

    gp3.setMu(s.maxDistance);
    gp3.setMaximumNearestNeighbors(s.maxNeighbors);
    gp3.setMaximumSurfaceAngle(s.maxSurfaceAngle); // 45 degrees
    gp3.setMinimumAngle(s.minAngle); // 10 degrees
    gp3.setMaximumAngle(s.maxAngle); // 120 degrees
    gp3.setNormalConsistency(s.normalConsistency);

	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

 	if (s.outputFile.substr(s.outputFile.find('.')) == ".vtk") 	
  		pcl::io::saveVTKFile(s.outputFile, triangles);
  	else if (s.outputFile.substr(s.outputFile.find('.')) == ".ply")
  		pcl::io::savePLYFile(s.outputFile, triangles);
  	else
  	{
  		std::cout << "Saving mesh as output.vtk\n";
  		pcl::io::saveVTKFile("output.vtk", triangles);
  	}
}

void savePoints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals, Settings s)
/* Saves Point Cloud and its Normals so it can be used with Smooth Signed Distance */
{
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::savePCDFileASCII("temp.pcd", *cloud_with_normals);

	pcl::io::loadPCDFile("temp.pcd", cloud_blob);
	pcl::io::savePLYFile(s.outputFile, cloud_blob, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false, false);
	remove("temp.pcd");
}

void convert(Settings s)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PCLPointCloud2 cloud_blob;
  	pcl::io::loadPCDFile (s.inputFile, cloud_blob);
  	pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);

	if (s.mesh)
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
		saveMesh(cloud_with_normals, s);
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);	
		pcl::copyPointCloud(*cloud, *cloud_with_normals);
		pcl::copyPointCloud(*normals, *cloud_with_normals);
		savePoints(cloud_with_normals, s);
	}
}

int main(int argc, char **argv)
{
	if (argc < 3)
	{
		printHelp();
		return 1;
	}

	Settings s;
	int c;
	static struct option long_options[] =
	{
		{"verbose", no_argument, 0, 'v'},
		{"mesh", no_argument, 0, 'm'},
		{"search-radius", required_argument, 0, 's'},
		{"max-distance", required_argument, 0, 'd'},
		{"max-neighbors", required_argument, 0, 'n'},
		{"minimum-angle", required_argument, 0, 'u'},
		{"maximum-angle", required_argument, 0, 'x'},
		{"normal-consistency", no_argument, 0, 'c'},
		{"surface-angle", required_argument, 0, 's'}
	};

	int option_index = 0;
	while ((c = getopt_long(argc, argv, "ms:d:n:u:x:cs:v", long_options, &option_index)) != -1)
	{
		std::cout << c << std::endl;
		switch(c)
		{
			case 's':
			s.searchRadius = std::atof(optarg);
			break;

			case 'd':
			s.maxDistance = std::atof(optarg);
			break;

			case 'n':
			s.maxNeighbors = std::atoi(optarg);
			break;

			case 'u':
			s.minAngle = std::atof(optarg);
			break;

			case 'x':
			s.maxAngle = std::atof(optarg);
			break;

			case 'c':
			s.normalConsistency = true;
			break;

			case 'm':
			s.mesh = true;
			break;

			case 'a':
			s.maxSurfaceAngle = std::atof(optarg);
			break;

			case 'v':
			s.verbose = true;
			break;

			case '?':
			std::cout << "Error: Invalid option\n";
			return 1;
			break;
		}
	}

	if ((argc - optind) >= 2)
	{
		s.inputFile = std::string(argv[optind++]);
		s.outputFile = std::string(argv[optind++]);
	}
	else
	{
		std::cout << "Error: Missing filenames\n";
		return 1;
	}

	if (s.verbose)
	{
		std::cout << "Mesh: " << s.mesh
			<< "\nSearch Radius " << s.searchRadius
			<< "\nMax Distance " << s.maxDistance
			<< "\nMax Nearest Neighbors " << s.maxNeighbors
			<< "\nMinimum Angle " << s.minAngle
			<< "\nMaximum Angle " << s.maxAngle
			<< "\nMaximum Surface Angle " << s.maxSurfaceAngle
			<< "\nNormal consistency " << s.normalConsistency << std::endl;
	}

	convert(s);

	return 0;
}
