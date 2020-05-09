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


def plotPath(path, plotter=plt, itr=-1, path_color='pink'):
	try:
		plotPoint(point=path[0].getXYCoords(), subplot_ax=plotter, radius=0.15, color='cyan')
		plotPoint(point=path[-1].getXYCoords(), subplot_ax=plotter, radius=0.15, color='magenta')
	except Exception:
		plotPoint(point=path[0], subplot_ax=plotter, radius=0.15, color='cyan')
		plotPoint(point=path[-1], subplot_ax=plotter, radius=0.15, color='magenta')

	prev_node = path[0]
	for node in path:
		try:
			pn_x, pn_y = prev_node.getXYCoords()
			cn_x, cn_y = node.getXYCoords()
		except Exception:
			pn_x, pn_y = prev_node
			cn_x, cn_y = node

		plotter.plot([pn_x, cn_x], [pn_y, cn_y], color=path_color, linewidth=3)

		if itr > -1:
			plt.savefig('./frames/%04d.png' % (itr))
			itr += 1

		prev_node = node

	return itr


# points in x,y format
def euclideanDistance(point_1, point_2):
	return np.sqrt(((point_1[0] - point_2[0]) ** 2) + ((point_1[1] - point_2[1]) ** 2))


def sameRegion(node1, node2, dist_thresh):
	return (euclideanDistance(node1.getXYCoords(), node2.getXYCoords()) < dist_thresh)


def convertNodeList2CoordList(node_list):
	return [node.getXYCoords() for node in node_list]
