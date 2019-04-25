from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import numpy as np
import argparse
import os
import math
import sys
import random
import scipy

from scipy.cluster.hierarchy import dendrogram, linkage
from matplotlib import pyplot as plt
import scipy.spatial.distance as ssd

libpcl = True
try:
	import pcl
	from pcl import pcl_visualization
except ImportError:
	print("Could not import python-pcl")
	import pclpy as pcl
	libpcl = False

def create_parser():
	parser = argparse.ArgumentParser(description='Shows similarities between histograms')

	parser.add_argument('input', help='Input histogram points file', type=str)
	parser.add_argument('inputPoints', help='Input xyz points file', type=str)

	return parser

#Histogram comparison functions

#1
def histogram_intersection(hist1, hist2):
	sum = 0
	for i in range(len(hist1)):
		sum += min(float(hist1[i]), float(hist2[i]))
	return sum
#2
def squared_euclidean_distance(hist1, hist2):
	sum = 0
	for i in range(len(hist1)):
		sum += pow(float(hist1[i]) - float(hist2[i]), 2)
	return sum
def load_histograms(filename):
	with open(filename, 'r') as fh:
		distance_mat = []
		histograms = []
		for line in fh:
			l = line[1:len(line)-2].split(',')
			histograms.append(l)
		for i in range(len(histograms)):
			distances = []
			for j in range(len(histograms)):
				if i == j: 
					distances.append(0)
				else:
					distances.append(squared_euclidean_distance(histograms[i],histograms[j]))
			distance_mat.append(distances);
	distArray = ssd.squareform(distance_mat) # distArray[{n choose 2}-{n-i choose 2} + (j-i-1)] is the distance between points i and j
	#plt.plot(distance_mat)
	#plt.show();
	Z = linkage(distArray, 'ward')
	fig = plt.figure(figsize=(25,10))
	dn = dendrogram(Z)
	plt.show()
	#print(dn['icoord'][len(dn['icoord'])-1])
	#print(dn['ivl'])
	print(Z)
	print(Z[0])
	print(Z[1])
	print(Z[len(Z)-1])
	return distance_mat



def main(argv):
	if not os.path.exists(argv.input):
		print("File {} does not exists".format(argv.input))
		sys.exit(1)

	if not argv.input.endswith('.txt') and not argv.input.endswith('.data'):
		print("File extension not recognized")
		sys.exit(1)

	if not os.path.exists(argv.inputPoints):
		print("File {} does not exists".format(argv.inputPoints))
		sys.exit(1)

	if not argv.inputPoints.endswith('.txt') and not argv.inputPoints.endswith('.data'):
		print("File extension not recognized")
		sys.exit(1)

	return load_histograms(argv.input)

if __name__ == '__main__':
	parser = create_parser()

	argv = parser.parse_args()

	main(argv)