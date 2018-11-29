from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import numpy as np

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

add_shape(cloud, shape):
	pass

def main(argv):
	cloud = load_pcd_normal(argv.input)

	cloud = add_shape(cloud, shape)

	save_pcd_normal(cloud, argv.output)

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Train the network')

	parser.add_argument('--input', '-i', required=True, type=str, 
		help='Input point cloud file')
	parser.add_argument('--output', '-o', type=str, default='output.pcd',
		help="Output file's name")
	parser.add_argument('--height', '-h', type=float, default=.25,
		help='Highest point in which shape can be extruded from')
	parser.add_argument('--verbose', action='store_true', default=False,
		help='Increase output verbosity')	
	argv = parser.parse_args()

	main(argv)
