from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import pcl
import numpy as np
import sys

argv = sys.argv

if len(argv) > 3:
	from noise import load_pcd_normal
	cloud_array = load_pcd_normal(argv[1])
else:
	cloud = pcl.load(argv[1])
	cloud_array = np.asarray(cloud)

mean = np.mean(cloud_array, axis=0)

for i in range(0, cloud_array.shape[0]):
	cloud_array[i][0] -= mean[0]
	cloud_array[i][1] -= mean[1]
	cloud_array[i][2] -= mean[2]

if len(argv) > 3:
	from noise import save_pcd_normal
	save_pcd_normal(cloud_array, argv[2])
else:
	cloud = pcl.PointCloud(cloud_array)
	pcl.save(cloud, argv[2], binary=False)
