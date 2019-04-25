
#include <string>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include "getopt.h"
#include <vector>
#include <fstream>
#include <random>

#include <pcl/search/impl/search.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/principal_curvatures.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

using namespace std;

Eigen::Vector4f    translation;
Eigen::Quaternionf orientation;

struct Settings
{
	std::string inputFile;  // Input filename
	std::string outputFile; // Output filename
	std::string outputPointsFile;
	double searchRadius;	// Search radius, default 0.025
	double maxDistance;		// Distance between points, default 2.5
	int maxNeighbors;		// Neighbors searched, default 100
	bool verbose;

	Settings() : searchRadius(150), maxDistance(2.5), maxNeighbors(100),
		verbose(false)
	{
		inputFile = "\0"; outputFile = "\0"; outputPointsFile = "\0";
	}
};

void printHelp()
{
	std::cout << "Basic usage: detection <input pcd file> <output file> <output points file>\n"
		<< "\t-r, --radius\t Search radius used to collect points,\n\t\t\tdefault 0.025\n"
		<< "\t-d, --max-distance\t Set max distance for points to be accepted,\n\t\t\tdefault 2.5\n"
		<< "\t-n, --max-neighbors\t Maximum Nearest Neighbors searched,\n\t\t\tdefault 100\n"
		<< "\t--verbose\t Set verbosity\n";
}


void
saveCloud(const std::string &filename, const pcl::PCLPointCloud2 &output)
{
	TicToc tt;
	tt.tic();

	print_highlight("Saving "); print_value("%s ", filename.c_str());

	io::savePCDFile(filename, output, translation, orientation, true);

	print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms : "); print_value("%d", output.width * output.height); print_info(" points]\n");
}

//histogram comparison functions

//#1
double histogram_intersection(FPFHSignature33 hist1, FPFHSignature33 hist2) {
	double sum = 0;
	for (int i = 0; i < 33; i++) {
		sum += std::min(hist1.histogram[i], hist2.histogram[i]);
	}
	return sum;
}

//#2
double squared_euclidean_distance(FPFHSignature33 hist1, FPFHSignature33 hist2) {
	double sum = 0;
	for (int i = 0; i < 33; i++) {
		sum += std::pow(hist1.histogram[i] - hist2.histogram[i], 2);
	}
	return sum;
}

/*
Next, the statistical 
x^2-test is examined in
its two forms
*/
//#3
double x1_test(FPFHSignature33 hist1, FPFHSignature33 hist2) {
	double sum = 0;
	for (int i = 0; i < 33; i++) {
		double top = std::pow(hist1.histogram[i] - hist2.histogram[i], 2);
		double bot = hist1.histogram[i];
		sum += top / bot;
	}
	return sum;
}

//#4
double x2_test(FPFHSignature33 hist1, FPFHSignature33 hist2) {
	double sum = 0;
	for (int i = 0; i < 33; i++) {
		double top = std::pow(hist1.histogram[i] - hist2.histogram[i], 2);
		double bot = hist1.histogram[i] + hist2.histogram[i];
		sum += top / bot;
	}
	return sum;
}

/*
Finally, we test the symmetric form of the Kullback-Leibler divergence
*/
//#5
double Kullback_divergence(FPFHSignature33 hist1, FPFHSignature33 hist2) {
	double sum = 0;
	for (int i = 0; i < 33; i++) {
		double first = hist2.histogram[i] - hist1.histogram[i];
		double second = log(hist2.histogram[i] / hist1.histogram[i]);
		sum += first * second;
	}
	return sum;
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

bool PointNormalEqual(PointNormal p1, PointNormal p2) {
	int dif = abs(p1.x - p2.x) == 0?0:1 + abs(p1.y - p2.y) == 0 ? 0 : 1 + abs(p1.z - p2.z) == 0 ? 0 : 1;
	cout<<(dif)<<endl;
	return dif == 0;
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
	// Estimate
	TicToc tt;
	tt.tic();

	print_highlight(stderr, "Computing ");

	pcl::PointCloud<pcl::PointXYZ>::Ptr xyzcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr ncloud(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PCLPointCloud2 cloud_blob;

	if (s.inputFile != "sphere") {
		if (pcl::io::loadPCDFile(s.inputFile, cloud_blob) == -1)
		{
			std::cout << "File no good" << std::endl;
			return 1;
		}
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cube(new pcl::PointCloud<pcl::PointNormal>);
		pcl::fromPCLPointCloud2(cloud_blob, *cloud_cube);

	}
	else {
		srand(time(0));
		int sphere_data_size = 300;
		cloud->width = sphere_data_size;
		cloud->height = 1;
		cloud->points.resize(cloud->width*cloud->height);
		default_random_engine generator;
		normal_distribution<double> distribution(5.0, 50.0);
		for (int i = 0; i < sphere_data_size; i++) {

			double x = distribution(generator);
			double y = distribution(generator);
			double z = distribution(generator);

			double x2 = x * (1 / sqrt(x*x + y * y + z * z)) * 1;
			double y2 = y * (1 / sqrt(x*x + y * y + z * z)) * 1;
			double z2 = z * (1 / sqrt(x*x + y * y + z * z)) * 1;

			/*double u1 = ((double)rand() / (RAND_MAX));
			double u2 = ((double)rand() / (RAND_MAX));


			double h = acos(2 * u1 - 1) - M_PI / 2;
			double o = 2 * M_PI * u2;

			double x = cos(h) * cos(o) * 10;
			double y = cos(h) * sin(o) * 10;
			double z = sin(h) * 10;*/

			cloud->points[i].x = x2;
			cloud->points[i].y = y2;
			cloud->points[i].z = z2;
		}
	}
	int size = cloud->points.size();

	split_cloud(cloud, xyzcloud, ncloud, size);

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(xyzcloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	/* Note: Should add if else once estimator is fixed
		for data that has and does not have normals */
		//estimate_curvature(xyzcloud, ncloud, s.searchRadius);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(xyzcloud);

	ne.setSearchMethod(tree);
	ne.setKSearch(s.maxNeighbors);
	//ne.setRadiusSearch(s.searchRadius);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.compute(*cloud_normals);
	/* end Note */

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> hist;
	hist.setInputCloud(xyzcloud);
	hist.setInputNormals(cloud_normals);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr treee(new pcl::search::KdTree<pcl::PointXYZ>);
	hist.setSearchMethod(treee);

	PointCloud<FPFHSignature33> fpfhs;
	hist.setRadiusSearch(s.searchRadius * 2);
	hist.compute(fpfhs);
	
	

	print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms : "); print_value("%d", fpfhs.width * fpfhs.height); print_info(" points]\n");

	// Convert data back
	pcl::PCLPointCloud2 output_fpfhs;
	pcl::toPCLPointCloud2(fpfhs, output_fpfhs);
	pcl::PCLPointCloud2 output;

	//concatenateFields(cloud_blob, output_fpfhs, output);
	//saveCloud(s.outputFile, output_fpfhs);
	cout << fpfhs.points[0] << endl;
	cout << endl;
	cout << fpfhs.points[1] << endl;
	cout << histogram_intersection(fpfhs.points[0], fpfhs.points[1]) << endl;
	cout << squared_euclidean_distance(fpfhs.points[0], fpfhs.points[1]) << endl;
	cout << x1_test(fpfhs.points[0], fpfhs.points[1]) << endl;
	cout << x2_test(fpfhs.points[0], fpfhs.points[1]) << endl;
	cout << Kullback_divergence(fpfhs.points[0], fpfhs.points[1]) << endl;

	ofstream myfile(s.outputFile);
	ofstream pointFile(s.outputPointsFile);

	if (myfile.is_open())
	{
		for (int i = 0; i < fpfhs.points.size(); ++i)
			myfile << fpfhs.points[i] << std::endl;
		myfile.close();
	}
	if (pointFile.is_open())
	{
		for (int i = 0; i < hist.getInputCloud()->size(); ++i)
			pointFile << hist.getInputCloud()->at(i) << std::endl;
		pointFile.close();
	}
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
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

	if ((argc - optind) >= 3)
	{
		s.inputFile = std::string(argv[optind++]);
		s.outputFile = std::string(argv[optind++]);
		s.outputPointsFile = std::string(argv[optind++]);
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