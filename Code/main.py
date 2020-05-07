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
import utils 
import obstacles as obs


X_LIM = (-5,5)
Y_LIM = (-5,5)

MAX_ITER = 5000
STEP_SIZE = 0.1
GOAL_REACH_THRESH = 0.5	

DRONE_RADIUS = 0.2


def rrtPlannedPath(start_node, goal_node, robot_radius, plotter):
	step_size = robot_radius * 2

	# rrt_nodes = [start_node]
	rrt_nodes = {start_node.getXYCoords(): start_node}
	
	# if plotter is not None:
	# 	utils.plotPoint(rand_node.getXYCoords(), plotter, radius=0.2, color='red')

	step_node = start_node

	itr = 0
	while (not utils.sameRegion(step_node, goal_node, GOAL_REACH_THRESH)) and (itr < MAX_ITER):
		# print("Iteration number:", itr)
		itr += 1

		# get random node in the direction of the goal node 40% of the times.
		rand_node = rrt.getRandomNode(x_lim=X_LIM, y_lim=Y_LIM, curr_node=start_node, goal_node=goal_node, goal_probability=0.6)
		if plotter is not None:
			utils.plotPoint(rand_node.getXYCoords(), plotter, radius=0.03, color='red')

		closest_node, _ = rrt.findClosestNode(rrt_nodes.values(), rand_node)

		step_node = rrt.getStepNode(closest_node, rand_node, step_size)
		if plotter is not None:
			utils.plotPoint(step_node.getXYCoords(), plotter, radius=0.04, color='blue')
			cn_x, cn_y = closest_node.getXYCoords()
			sn_x, sn_y = step_node.getXYCoords()
			plotter.plot([cn_x, sn_x], [cn_y, sn_y], color='blue')
			# plt.show()
			# plt.pause(0.5)

		if rrt.hitsObstacle(start_node=closest_node, goal_node=step_node, step_size=(robot_radius/2)):
			if plotter is not None:
				utils.plotPoint(step_node.getXYCoords(), plotter, radius=0.04, color='red')
				cn_x, cn_y = closest_node.getXYCoords()
				sn_x, sn_y = step_node.getXYCoords()
				plotter.plot([cn_x, sn_x], [cn_y, sn_y], color='red')
				# plt.show()
				# plt.pause(0.5)
			continue

		if plotter is not None:
			utils.plotPoint(step_node.getXYCoords(), plotter, radius=0.04, color='green')
			cn_x, cn_y = closest_node.getXYCoords()
			sn_x, sn_y = step_node.getXYCoords()
			plotter.plot([cn_x, sn_x], [cn_y, sn_y], color='green')
			# plt.show()
			# plt.pause(0.5)

		rrt_nodes.update({step_node.getXYCoords(): step_node})

		if plotter is not None:
			plt.show()
			plt.pause(0.05)
			plt.savefig('./frames/' + str(itr) + '.png')

	# Reached Goal
	if utils.sameRegion(step_node, goal_node, GOAL_REACH_THRESH):
		print("Reached Goal!")
		print("Number of iterations:", itr)

		goal_node.parent_coords = closest_node.getXYCoords()
		goal_node.distance = utils.euclideanDistance(point_1=closest_node.getXYCoords(), point_2=goal_node.getXYCoords())
		rrt_nodes.update({goal_node.getXYCoords(): goal_node})
		# rrt_nodes.append(goal_node)

		path = rrt.backtrack(rrt_nodes, goal_node)

		print("path:", len(path), "rrt_nodes:", len(rrt_nodes))

		return (path, rrt_nodes, itr)

	return (None, None, None)


def main():
	start_node = node.Node(current_coords=(-4, -4), parent_coords=None, distance=0)
	goal_node = node.Node(current_coords=(4, 4), parent_coords=None, distance=0)

	fig, ax = plt.subplots()
	ax.set_xlim(-6, 6)
	ax.set_ylim(-6, 6)
	fig.gca().set_aspect('equal', adjustable='box')
	utils.plotPoint(start_node.getXYCoords(), ax, radius=0.06, color='cyan') 	# start
	utils.plotPoint(goal_node.getXYCoords(), ax, radius=0.06, color='magenta') 	# end

	obs.generateMap(ax)

	plt.ion()
	rrt_path, _, itr = rrtPlannedPath(start_node, goal_node, robot_radius=DRONE_RADIUS, plotter=ax)
	if rrt_path is not None:
		utils.plotPath(rrt_path, plotter=ax, itr=itr)
	plt.ioff()
	plt.show()

	rrt_path_coords = utils.convertNodeList2CoordList(node_list=rrt_path)

	np.save(file='rrt_path_nodes.npy', arr=rrt_path)
	np.save(file='rrt_path_coords.npy', arr=rrt_path_coords)

if __name__ == '__main__':
	main()