from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import numpy as np
import pcl
from pcl import pcl_visualization
import argparse
import os
import sys
import random
import math

def create_parser():
	parser = argparse.ArgumentParser(description='Adds synthetic noise to point cloud')

	parser.add_argument('input', help='Input point cloud file', type=str)
	parser.add_argument('output', help='Output point cloud file', type=str)
	parser.add_argument('-d', '--distance', help='Max distance that a point will move', type=float,
		default=1, action='store', dest='distance')
	parser.add_argument('-p', '--probability', help='Probability to change coordinates of point', type=float,
		default=0.01, action='store', dest='probability')
	parser.add_argument('-v', '--visualize', help='Visualize the point cloud after adding noise',
		action='store_true', default=False, dest='visualize')
	parser.add_argument('-c', '--color', help='Colors the points that are modified',
		action='store_true', default=False, dest='color')
	parser.add_argument('-n', '--normal', help='Use the normal to add noise',
	 	action='store_true', default=False, dest='normal')
	parser.add_argument('-g', '--grid', help='Use a voxel grid to add noise',
	 	action='store_true', default=False, dest='grid')
	parser.add_argument('--verbose', help='Increase program verbosity', action='store_true', 
		default=False)

	return parser

def load_pcd_normal(filename):
	with open(filename, 'r') as fh:
		cloud = []
		flag = True
		i = 0

		for line in fh:
			l = line.split(' ')
			if flag and l[0].upper() == 'POINTS':
				cloud = np.zeros((int(l[1]), 7), dtype=np.float32)
			elif flag and l[0].upper() == 'DATA':
				flag = False
			elif not flag:
				cloud[i][0] = float(l[0])
				cloud[i][1] = float(l[1])
				cloud[i][2] = float(l[2])
				cloud[i][4] = float(l[3])
				cloud[i][5] = float(l[4])
				cloud[i][6] = float(l[5])
				i += 1

				if i == cloud.shape[0]: break

	return cloud

def save_pcd_normal(cloud, filename):
	header = '''# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z normal_x normal_y normal_z
SIZE 4 4 4 4 4 4
TYPE F F F F F F
COUNT 1 1 1 1 1 1
WIDTH {0}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {0}
DATA ascii
'''.format(cloud.shape[0])

	content = ""
	for row in cloud:
		line = "{} {} {} {} {} {}\n".format(row[0], row[1], row[2], 
			row[4], row[5], row[6])
		content += line

	pcd = header + content

	with open(filename, 'w') as pcd_file:
			pcd_file.write(pcd)

def add_uniform(cloud, argv):
	count = 0

	for i in range(0, cloud.shape[0]):
		if random.random() <= argv.probability:
			#cloud[i] += (dist * (-1)**random.randint(1, 100))
			cloud[i][0] += (random.random() * argv.distance * (-1)**random.randint(1, 100))
			cloud[i][1] += (random.random() * argv.distance * (-1)**random.randint(1, 100))
			cloud[i][2] += (random.random() * argv.distance * (-1)**random.randint(1, 100))
			if argv.color: cloud[i][3] = 255 << 16 | 0 << 8 | 0
			count += 1
	return cloud, count

def move_along_normals(cloud, argv):
	count = 0
	mean = np.mean(cloud, axis=0)

	for i in range(0, cloud.shape[0]):
		if random.random() <= argv.probability:
			old = np.array([cloud[i][0], cloud[i][1], cloud[i][2]])

			cloud[i][0] += (random.random() * argv.distance * cloud[i][4])
			cloud[i][1] += (random.random() * argv.distance * cloud[i][5])
			cloud[i][2] += (random.random() * argv.distance * cloud[i][6])
			new = np.array([cloud[i][0], cloud[i][1], cloud[i][2]])

			#TODO: Should probably replace with something better
			if math.sqrt((mean[0] - new[0])**2 + (mean[1] - new[1])**2 + (mean[2] - new[2])**2) \
				< math.sqrt((mean[0] - old[0])**2 + (mean[1] - old[1])**2 + (mean[2] - old[2])**2):
				cloud[i][0] += (random.random() * argv.distance * -1 * cloud[i][4])
				cloud[i][1] += (random.random() * argv.distance * -1 * cloud[i][5])
				cloud[i][2] += (random.random() * argv.distance * -1 * cloud[i][6])	

			if argv.color: cloud[i][3] = 255 << 16 | 0 << 8 | 0
			count += 1
	return cloud, count

def voxel_noise(cloud, argv):
	pass

def add_noise(cloud, argv):
	if argv.verbose: print("Adding noise to point cloud by ", end='')
	if argv.grid:
		if argv.verbose: print('using a voxel grid.')
		cloud, count = voxel_noise(cloud, argv)
	elif argv.normal:
		if argv.verbose: print('moving points along their normals.')
		cloud, count = move_along_normals(cloud, argv)
	else:
		if argv.verbose: print('moving points a random distance and direction.')
		cloud, count = add_uniform(cloud, argv)
	if argv.verbose: print(str(count) + " points changed.")
	return cloud

def color_cloud(cloud):
	#colored_cloud = pcl.PointCloud_PointXYZRGB()

	if cloud.shape[1] < 4:
		cloud = np.hstack((cloud, np.zeros((cloud.shape[0], 1), 
			dtype=np.float32)))

	for i in range(0, cloud.shape[0]):
		cloud[i][3] = 255 << 16 | 255 << 8 | 0

	#colored_cloud.from_array(cloud)
	return cloud #colored_cloud

def main(argv):
	if (argv.verbose):
		print('''
Input File:     {}
Output File:    {}
Distance:       {}
Probability:    {}
View Cloud:     {}
Color Noise:    {}
Use Normal:     {}
Use Voxel Grid: {}
			'''.format(argv.input, argv.output, argv.distance, argv.probability,
			 argv.visualize, argv.color, argv.normal, argv.grid))

	if argv.normal and argv.grid:
		print("Note: Only one method will used.")
	
	if not os.path.exists(argv.input):
		print("File {} does not exists".format(argv.input))
		sys.exit(1)

	if not argv.input.endswith('.pcd') or not argv.output.endswith('.pcd'):
		print("File extension not recognized")
		sys.exit(1)

	if argv.normal:
		cloud_array = load_pcd_normal(argv.input)
	else:
		cloud = pcl.load(argv.input)
		cloud_array = np.asarray(cloud)

	cloud_array = color_cloud(cloud_array)
	cloud_array = add_noise(cloud_array, argv)

	if argv.verbose: print("Saving Point Cloud")
	if argv.normal:
		#save_pcd_normal(cloud_array, argv.output)
		cloud = pcl.PointCloud_PointXYZRGB(np.delete(cloud_array, np.s_[4:], axis=1))
		cloud_save = pcl.PointCloud(np.delete(cloud_array, np.s_[3:], axis=1))
		pcl.save(cloud_save, argv.output, binary=False)
	else:
		cloud = pcl.PointCloud_PointXYZRGB(cloud_array)
		cloud_save = pcl.PointCloud(np.delete(cloud_array, 3, axis=1))
		pcl.save(cloud_save, argv.output, binary=False)

	if argv.visualize:
		if argv.verbose: print("Opening Cloud Viewer; for help press 'h' from within the viewer")
		print(cloud.size)
		#cloud = color_cloud(cloud_array, argv)
		visual = pcl_visualization.CloudViewing()
		visual.ShowColorCloud(cloud, b'cloud')

		while not visual.WasStopped(): pass


if __name__ == '__main__':
	parser = create_parser()

	argv = parser.parse_args()

	main(argv)

'''
Begin Function CalculateSurfaceNormal (Input Triangle) Returns Vector

	Set Vector U to (Triangle.p2 minus Triangle.p1)
	Set Vector V to (Triangle.p3 minus Triangle.p1)

	Set Normal.x to (multiply U.y by V.z) minus (multiply U.z by V.y)
	Set Normal.y to (multiply U.z by V.x) minus (multiply U.x by V.z)
	Set Normal.z to (multiply U.x by V.y) minus (multiply U.y by V.x)

	Returning Normal

End Function
'''
