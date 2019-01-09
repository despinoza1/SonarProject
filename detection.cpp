
#include <string>
#include <cstdlib>
#include <iostream>
#include <getopt.h>
#include <vector>

#include <pcl/search/impl/search.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/principal_curvatures.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct Settings
{
	std::string inputFile;  // Input filename
	std::string outputFile; // Output filename
	double searchRadius;	// Search radius, default 0.025
	double maxDistance;		// Distance between points, default 2.5
	int maxNeighbors;		// Neighbors searched, default 100
	bool verbose;

	Settings() : searchRadius(150), maxDistance(2.5), maxNeighbors(100),
		verbose(false)
		{ inputFile = "\0"; outputFile = "\0"; }
};

void printHelp()
{
	std::cout << "Basic usage: detection <input pcd file> <output file>\n"
		<< "\t-r, --radius\t Search radius used to collect points,\n\t\t\tdefault 0.025\n"
		<< "\t-d, --max-distance\t Set max distance for points to be accepted,\n\t\t\tdefault 2.5\n"
		<< "\t-n, --max-neighbors\t Maximum Nearest Neighbors searched,\n\t\t\tdefault 100\n"
		<< "\t--verbose\t Set verbosity\n";
}

void estimate_curvature(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzcloud, 
	pcl::PointCloud<pcl::Normal>::Ptr ncloud,
	int k)
{
	/*
		Need to look up PCL Feature Stuff for proper template
	*/
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

  	principalCurvaturesEstimation.setInputCloud(xyzcloud);
  	principalCurvaturesEstimation.setInputNormals(ncloud);

 	principalCurvaturesEstimation.setSearchMethod(tree);
  	principalCurvaturesEstimation.setKSearch(k);

  	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
	principalCurvaturesEstimation.compute(*principalCurvatures);

	for (int i = 0; i < principalCurvatures->points.size(); ++i)
	{
		pcl::PrincipalCurvatures descriptor = principalCurvatures->points[i];
		std::cout << descriptor.pc1 << " " << descriptor.pc2 << std::endl;
	}
}

void split_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyzcloud,
	pcl::PointCloud<pcl::Normal>::Ptr ncloud,
	int size)
{
	xyzcloud->width = size;
  	ncloud->width = size;

  	xyzcloud->height = 1;
  	ncloud->height = 1;

  	xyzcloud->points.resize(size);
  	ncloud->points.resize(size);

  	for (int i = 0; i < size; i++)
  	{
  		xyzcloud->points[i].x = cloud->points[i].x;
  		xyzcloud->points[i].y = cloud->points[i].y;
  		xyzcloud->points[i].z = cloud->points[i].z;

  		ncloud->points[i].normal_x = cloud->points[i].normal_x;
  		ncloud->points[i].normal_y = cloud->points[i].normal_y;
  		ncloud->points[i].normal_z = cloud->points[i].normal_z;
 		ncloud->points[i].curvature = cloud->points[i].curvature;
  	}
}

int fpfh(Settings s)
{
	/*
	Need Curvature
	Split XYZ and Normals + Curvature to two clouds
	Compute FPFH
	Get basic geometric shapes from FPFH
	Maybe label them by color
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyzcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr ncloud(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PCLPointCloud2 cloud_blob;
  	
  	if (pcl::io::loadPCDFile (s.inputFile, cloud_blob) == -1)
  	{
  		std::cout << "File no good" << std::endl;
  		return 1;
  	}
  	
  	pcl::fromPCLPointCloud2(cloud_blob, *cloud);

  	int size = cloud->points.size();

  	split_cloud(cloud, xyzcloud, ncloud, size);

/* Note: Should add if else once estimator is fixed
	for data that has and does not have normals */
	//estimate_curvature(xyzcloud, ncloud, s.searchRadius);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  	ne.setInputCloud (xyzcloud);

  	ne.setSearchMethod(tree);
  	ne.setKSearch(s.maxNeighbors);
  	//ne.setRadiusSearch(s.searchRadius);
 	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  	ne.compute(*cloud_normals);
/* end Note */

  	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> hist;
	hist.setInputCloud(xyzcloud);
  	hist.setInputNormals(cloud_normals);

  	pcl::search::KdTree<pcl::PointXYZ>::Ptr treee (new pcl::search::KdTree<pcl::PointXYZ>);
  	hist.setSearchMethod(treee);

  	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh (new pcl::PointCloud<pcl::FPFHSignature33> ());
  	hist.setRadiusSearch(s.searchRadius*2);
  	hist.compute(*fpfh);

  	for (int i = 0; i < fpfh->points.size(); ++i)
  		std::cout << fpfh->points[i] << std::endl;

	return 0;
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
		{"radius", required_argument, 0, 'r'},
		{"max-distance", required_argument, 0, 'd'},
		{"max-neighbors", required_argument, 0, 'n'},
	};

	int option_index = 0;
	while ((c = getopt_long(argc, argv, "r:d:n:v", long_options, &option_index)) != -1)
	{
		switch(c)
		{
			case 'r':
			s.searchRadius = std::atof(optarg);
			break;

			case 'd':
			s.maxDistance = std::atof(optarg);
			break;

			case 'n':
			s.maxNeighbors = std::atoi(optarg);
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
		std::cout << "\nSearch Radius " << s.searchRadius
			<< "\nMax Distance " << s.maxDistance
			<< "\nMax Nearest Neighbors " << s.maxNeighbors << std::endl;
	}

	return fpfh(s);
}