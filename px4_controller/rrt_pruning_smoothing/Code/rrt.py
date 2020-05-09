from __future__ import print_function, division 
import sys
import math
import heapq
import random 
import numpy as np 
import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

# import main_no_prune as main
import main
import node 
# import univ
import utils 
import obstacles as obs


def getRandomNode(x_lim, y_lim, curr_node, goal_node, goal_probability=0.5):
	prob = np.random.sample()
	
	# exploration
	if prob > goal_probability:
		x_lim_low, x_lim_high = x_lim
		y_lim_low, y_lim_high = y_lim
	
	# heading towards to goal
	else:
		gn_x, gn_y = goal_node.getXYCoords()
		cn_x, cn_y = curr_node.getXYCoords()

		if (gn_y - cn_y) > 0:
			y_lim_low = cn_y
			y_lim_high = gn_y + ((y_lim[1] - gn_y) / 3)
		else:
			y_lim_low = gn_y - ((gn_y - y_lim[0]) / 3)
			y_lim_high = cn_y

		if (gn_x - cn_x) > 0:
			x_lim_low = cn_x
			x_lim_high = gn_x + ((x_lim[1] - gn_x) / 3)
		else:
			x_lim_low = gn_x - ((gn_x - x_lim[0]) / 3)
			x_lim_high = cn_x

	rn_x = np.random.uniform(low=x_lim_low, high=x_lim_high)
	rn_y = np.random.uniform(low=y_lim_low, high=y_lim_high)

	return node.Node(current_coords=(rn_x, rn_y), parent_coords=None, distance=0)


def getStepNode(closest_node, rand_node, step_size):
	# dist = utils.euclideanDistance(point_1=closest_node.getXYCoords(), point_2=rand_node.getXYCoords())

	cn_x, cn_y = closest_node.getXYCoords()
	rn_x, rn_y = rand_node.getXYCoords()

	# slope = (rn_y - cn_y) / (rn_x - cn_x)
	theta = np.arctan2((rn_y - cn_y), (rn_x - cn_x))

	# intercept = cn_y - (slope * cn_x)

	sn_x = cn_x + (step_size * np.cos(theta))
	sn_y = cn_y + (step_size * np.sin(theta))

	return node.Node(current_coords=(sn_x, sn_y), parent_coords=(cn_x, cn_y), distance=step_size)


def hitsObstacle(start_node, goal_node, step_size=0.1):
	sn_x, sn_y = start_node.getXYCoords()
	gn_x, gn_y = goal_node.getXYCoords()

	theta = np.arctan2((gn_y - sn_y), (gn_x - sn_x))

	nn_x, nn_y = sn_x, sn_y
	while (utils.euclideanDistance(point_1=(nn_x, nn_y), point_2=(gn_x, gn_y)) > 0.001):
		# next node
		nn_x = nn_x + (step_size * np.cos(theta))
		nn_y = nn_y + (step_size * np.sin(theta))

		if obs.withinObstacleSpace(point=(nn_x, nn_y), radius=main.DRONE_RADIUS, clearance=main.DRONE_RADIUS / 2):
			return True

	return False


def findClosestNode(rrt_nodes, rand_node):
	closest_dist = 99999
	closest_node = None

	for rrt_node in rrt_nodes:
		node_dist = utils.euclideanDistance(point_1=rrt_node.getXYCoords(), point_2=rand_node.getXYCoords())

		if node_dist < closest_dist:
			closest_dist = node_dist
			closest_node = rrt_node

	return (closest_node, closest_dist)


def backtrack(rrt_nodes, goal_node):
	path = []

	temp_node = goal_node
	while (temp_node.parent_coords is not None):
		path.insert(0, temp_node)
		temp_node = rrt_nodes[temp_node.parent_coords]

	path.insert(0, temp_node)

	return path


def prunePath(path_list):
	pass