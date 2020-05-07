from __future__ import print_function, division 
import sys
import math
import heapq
import random 
import numpy as np 
import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

import node 
# import univ
import utils 
# import obstacles


def getRandomNode(x_lim, y_lim, curr_node, goal_node, goal_probability=0.4):
	prob = np.random.sample()
	
	if prob > goal_probability:
		x_lim_low, x_lim_high = x_lim
		y_lim_low, y_lim_high = y_lim
	else:
		gn_x, gn_y = goal_node.getXYCoords()
		cn_x, cn_y = curr_node.getXYCoords()

	rn_x = np.random.uniform(low=x_lim[0], high=x_lim[1])
	rn_y = np.random.uniform(low=y_lim[0], high=y_lim[1])




def getStepNode(closest_node, rand_node, step_size):
	# dist = utils.euclideanDistance(point_1=closest_node.getXYCoords(), point_2=rand_node.getXYCoords())

	cn_x, cn_y = closest_node
	rn_x, rn_y = rand_node

	# slope = (rn_y - cn_y) / (rn_x - cn_x)
	theta = np.arctan2((rn_y - cn_y), (rn_x - cn_x))

	# intercept = cn_y - (slope * cn_x)

	sn_x = cn_x + (step_size * np.cos(theta))
	sn_y = cn_y + (step_size * np.sin(theta))

	return node.Node(current_coords=(sn_x, sn_y), parent_coords=(cn_x, cn_y), distance=step_size)