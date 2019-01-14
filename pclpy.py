import numpy as np

def save(cloud, path, binary=False):
	'''Saves a point cloud with xyz data'''
	if binary:
		print("Can only open ascii files")
		return np.empty(1)

	cloud = cloud.get_cloud()

	header = '''# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z 
SIZE 4 4 4 
TYPE F F F 
COUNT 1 1 1 
WIDTH {0}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {0}
DATA ascii
'''.format(cloud.shape[0])

	content = ""
	for row in cloud:
		line = "{} {} {}\n".format(row[0], row[1], row[2])
		content += line

	pcd = header + content

	with open(path, 'w') as pcd_file:
			pcd_file.write(pcd)

def load(path):
	'''Loads a point cloud with xyz data'''
	with open(path, 'r') as fh:
		cloud = []
		flag = True
		i = 0

		for line in fh:
			l = line.split(' ')
			if flag and l[0].upper() == 'POINTS':
				cloud = np.zeros((int(l[1]), 3), dtype=np.float32)
			elif flag and l[0].upper() == 'DATA':
				flag = False
			elif not flag:
				cloud[i][0] = float(l[0])
				cloud[i][1] = float(l[1])
				cloud[i][2] = float(l[2])
				i += 1

				if i == cloud.shape[0]: break

	return PointCloud(cloud)

class PointCloud:
	'''XYZ Point Cloud'''
	def __init__(self, cloud=np.empty(1)):
		self.cloud = np.array(cloud)

	def get_cloud(self):
		return self.cloud

	def __str__(self):
		return "Point Cloud of size %d" % self.cloud.shape[0]

class PointCloud_PointXYZRGB:
	'''XYZ and RGB Point Cloud'''
	def __init__(self, cloud=np.empty(1)):
		self.cloud = np.array(cloud)

	def get_cloud(self):
		return self.cloud

	def __str__(self):
		return "Point Cloud XYZRGB of size %d" % self.cloud.shape[0]
