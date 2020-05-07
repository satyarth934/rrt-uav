from __future__ import print_function, division
# import os
import sys
import numpy as np
import matplotlib.pyplot as plt


def plotPoint(point, subplot_ax, radius=0.2, color='black'):
	dot = plt.Circle(point, radius=radius, color=color)
	subplot_ax.add_artist(dot)


def plotPoints(points_list, subplot_ax, radius=0.2, color='black'):
	for point in points_list:
		plotPoint(point, subplot_ax, radius, color)
		# dot = plt.Circle(point, radius=radius, color=color)
		# subplot_ax.add_artist(dot)


# points in x,y format
def euclideanDistance(point_1, point_2):
	return np.sqrt(((point_1[0] - point_2[0]) ** 2) + ((point_1[1] - point_2[1]) ** 2))