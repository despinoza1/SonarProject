from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

#from liblas import file
from laspy.file import File
import numpy as np
import sys
import csv

def scale_dimension(value, scale, offset):
	return (value * scale + offset)

def main(argv):
	if len(argv) < 2:
		print('''Usage: {} input.{} output.pcd
Converts LAS or XYZ file to Point Cloud Data'''.format(sys.argv[0], '{xyz, las, txt, csv}'))
		sys.exit(1)

	content = ""
	nPoints = 0
	flag = True

	if (argv[1].endswith('.las')):
		print('Reading LAS file')
		las_file = File(argv[1], mode='r')
		
		offset  = las_file.header.offset
		scale   = las_file.header.scale
		points  = las_file.points
		nPoints = points.size

		for p in points:
		    content += "{} {} {}\n".format(scale_dimension(p[0][0], scale[0], offset[0]),
		    	scale_dimension(p[0][1], scale[1], offset[1]),
		    	scale_dimension(p[0][2], scale[2], offset[2]) )

		las_file.close()

	elif (argv[1].endswith('.txt') or argv[1].endswith('.csv')):
		print('Reading ASCII lidar data')

		with open(argv[1], 'r') as ascii_lidar:
			lidar_reader = csv.DictReader(ascii_lidar, delimiter=',')
			for row in lidar_reader:
				content += '{} {} {}\n'.format(row['x'], row['y'], row['z'])
				nPoints += 1

	elif (argv[1].endswith('.xyz')):
		print('Reading XYZ data')

		with open(sys.argv[1], 'r') as xyz_file:
			content = xyz_file.read()

		nPoints = sum(1 for line in open(sys.argv[1], 'r'))
		flag = False

	else:
		print('File {} not supported'.format(argv[1]))
		sys.exit(1)


	header = '''# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z {1}
SIZE 4 4 4 {2}
TYPE F F F {3}
COUNT 1 1 1 {4}
WIDTH {0}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {0}
DATA ascii
'''.format(nPoints, "" if flag else "normal_x normal_y normal_z",
	"" if flag else "4 4 4", "" if flag else "F F F", "" if flag else "1 1 1")

	pcd = header + content

	with open(argv[2], 'w') as pcd_file:
			pcd_file.write(pcd)

if __name__ == '__main__':
	main(sys.argv)
