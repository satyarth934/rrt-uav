from __future__ import print_function, division 
import sys
import math
import heapq
import random 
import numpy as np 
import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

import rrt
import node
# import univ
# import utils 
# import obstacles


X_LIM = (-5,5)
Y_LIM = (-5,5)

MAX_ITER = 5000
STEP_SIZE = 0.1
GOAL_REACH_THRESH = 0.1	


def rrtPlannedPath(start_node, goal_node):
	step_size = radius * 5

	rrt_node_list = [start_node]

	# get random node in the direction of the goal node 40% of the times.
	rand_node = rrt.getRandomNode(x_lim=X_LIM, y_lim=Y_LIM, goal_probability=0.4)
	
	closest_node = start_node

	itr = 0
	while (not withinGoalRegion(closest_node)) and (itr < MAX_ITER):
		step_node = rrt.getStepNode(closest_node, rand_node, step_size)

		if hitsObstacle(start_node=closest_node, goal_node=step_node, step_size=(radius/2)):
			continue

		rrt_node_list.append(step_node)

		rand_node = getRandomNode(goal_dir_probability=0.4)

		closest_node = findClosestNode(rrt_node_list, rand_node)

		itr += 1

	# Reached Goal
	if withinGoalRegion(closest_node):
		plotPath()


def main():
	start_node = node.Node(current_coords=(-4, -4), parent_coords=None, distance=0)
	goal_node = node.Node(current_coords=(0, -4), parent_coords=None, distance=0)
	rrt_path = rrtPlannedPath(start_node, goal_node)


if __name__ == '__main__':
	main()