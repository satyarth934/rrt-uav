from __future__ import print_function, division
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

import main as mn
import utils
import obstacles as obs
import bezier_curves as bc


def bezierCurveTesting0():
	P0 = (1, 1)
	P1 = (10, 20)
	P2 = (15, 18)
	P3 = (17, 8)

	bcc_x, bcc_y = bc.bezierCubicCurveEquation(P0, P1, P2, P3, step_size=0.01)
	bqc_x, bqc_y = bc.bezierQuadraticCurveEquation(P0, P1, P2, step_size=0.01)

	p0_dot = plt.Circle(P0, radius=0.2, color='black')
	p1_dot = plt.Circle(P1, radius=0.2, color='black')
	p2_dot = plt.Circle(P2, radius=0.2, color='black')
	p3_dot = plt.Circle(P3, radius=0.2, color='black')

	fig, ax = plt.subplots()
	ax.set_xlim(-1, 25)
	ax.set_ylim(-1, 25)

	ax.add_artist(p0_dot)
	ax.add_artist(p1_dot)
	ax.add_artist(p2_dot)
	ax.add_artist(p3_dot)

	plt.plot(bcc_x, bcc_y, color='green')
	plt.plot(bqc_x, bqc_y, color='red')
	plt.show()


def bezierCurveTesting1(point_list, animate=False, write_path=None):

	# fig, ax = plt.subplots()
	# fig.gca().set_aspect('equal', adjustable='box')
	# ax.set_xlim(-6, 6)
	# ax.set_ylim(-6, 6)

	# obs.generateMap(plotter=ax)

	# utils.plotPoints(point_list, ax, radius=0.04, color='black')		# all points
	# utils.plotPoint(point_list[0], ax, radius=0.06, color='cyan') 		# start
	# utils.plotPoint(point_list[-1], ax, radius=0.06, color='magenta') 	# end

	# List of all path points
	BC_x = []
	BC_y = []

	# Initial points
	try:
		P0 = point_list[0]
		P1 = point_list[1]
		P2 = point_list[2]
		P3 = point_list[3]
	except Exception:
		print("Insufficient points. Appropriate case handled later.")

	# Starting Curve
	if len(point_list) > 4:
		bc_x, bc_y = bc.bezierCubicCurveEquation(P0, P1, P2, ((P2 + P3) / 2), step_size=0.01)
	elif len(point_list) == 4:
		bc_x, bc_y = bc.bezierCubicCurveEquation(P0, P1, P2, P3, step_size=0.01)
	elif len(point_list) == 3:
		bc_x, bc_y = bc.bezierQuadraticCurveEquation(P0, P1, P2, step_size=0.01)
	elif len(point_list) == 2:
		bc_x, bc_y = bc.bezierLinearCurveEquation(P0, P1, step_size=0.01)

	BC_x.extend(bc_x)
	BC_y.extend(bc_y)

	# Handling the rest of the points, leaving out the last 1 or 2 points for the end
	point_idx = 4
	if point_idx >= len(point_list):
		return (BC_x, BC_y)

	while point_idx + 2 < len(point_list):
		P0 = P2
		P1 = P3
		P2 = point_list[point_idx]
		point_idx += 1

		P3 = point_list[point_idx]
		point_idx += 1

		bc_x, bc_y = bc.bezierCubicCurveEquation(((P0 + P1) / 2), P1, P2, ((P2 + P3) / 2), 0.01)
		BC_x.extend(bc_x)
		BC_y.extend(bc_y)

	# Ending Curve
	P0 = P2
	P1 = P3
	P2 = point_list[point_idx]
	point_idx += 1

	if point_idx < len(point_list):
		P3 = point_list[point_idx]
		bc_x, bc_y = bc.bezierCubicCurveEquation(((P0 + P1) / 2), P1, P2, P3, step_size=0.01)
	else:
		bc_x, bc_y = bc.bezierQuadraticCurveEquation(((P0 + P1) / 2), P1, P2, step_size=0.01)

	BC_x.extend(bc_x)
	BC_y.extend(bc_y)

	return (BC_x, BC_y)


def findClosestWayPoint(ref_point, point_list):
	rx, ry = ref_point

	min_dist = 99999
	closest_wp_idx = -1

	for i in range(len(point_list)):
		dist = utils.euclideanDistance(point_1=ref_point, point_2=point_list[i])
		if dist < min_dist:
			min_dist = dist
			closest_wp_idx = i

	return (min_dist, closest_wp_idx)


def visualizeNewPath(BC_x, BC_y, point_list, animate=False, write_path=None, itr=-1):
	fig, ax = plt.subplots()
	fig.gca().set_aspect('equal', adjustable='box')
	ax.set_xlim(-6, 6)
	ax.set_ylim(-6, 6)

	obs.generateMap(plotter=ax)

	utils.plotPoints(point_list, ax, radius=0.04, color='black')		# all points
	utils.plotPoint(point_list[0], ax, radius=0.06, color='cyan') 		# start
	utils.plotPoint(point_list[-1], ax, radius=0.06, color='magenta') 	# end

	utils.plotPath(path=point_list, plotter=ax, path_color='black')
	utils.plotPath(path=zip(BC_x, BC_y), plotter=ax, path_color='lime')

	if write_path is not None:
		if itr > -1:
			plt.savefig(os.path.join(write_path, ('%04d.png' % itr)))
			itr += 1
			return itr
		else:
			plt.savefig(os.path.join(write_path, ('frame.png')))

	plt.show()
	if animate:
		plt.pause(0.5)


def main_no():
	# point_list = np.random.randint(0, 100, (10, 2))
	# print(point_list)
	# point_list.sort(axis=0)
	# print(point_list)
	point_list = np.load('rrt_prune_smooth_path_coords.npy')
	print("original list:", len(point_list))

	(BC_x, BC_y) = bezierCurveTesting1(point_list, animate=True, write_path='./rrt_prune_smooth_frames')

	# for bcx, bcy in zip(BC_x, BC_y):
	idx = 0
	while idx < len(BC_x):
		bcx, bcy = BC_x[idx], BC_y[idx]

		# if the current waypoint is within the obstacle space => make a new waypoint
		if obs.withinObstacleSpace(point=(bcx, bcy), radius=mn.DRONE_RADIUS, clearance=(mn.DRONE_RADIUS / 2)):

			# find the 2 closest original waypoints
			_, closest_wp_idx = findClosestWayPoint(ref_point=(bcx, bcy), point_list=point_list)

			if closest_wp_idx > 0:
				prev_cwp = point_list[closest_wp_idx - 1]
			else:
				prev_cwp = None

			if closest_wp_idx < len(point_list) - 1:
				next_cwp = point_list[closest_wp_idx + 1]
			else:
				next_cwp = None

			if prev_cwp is None and next_cwp is None:
				continue
			elif prev_cwp is not None and next_cwp is None:
				other_wp_idx = closest_wp_idx - 1
				other_cwp = prev_cwp
			elif next_cwp is not None and prev_cwp is None:
				other_wp_idx = closest_wp_idx + 1
				other_cwp = next_cwp
			else:
				_, other_wp_idx = findClosestWayPoint(ref_point=(bcx, bcy), point_list=[prev_cwp, next_cwp])

				if other_wp_idx == 0:
					other_wp_idx = closest_wp_idx - 1
					other_cwp = prev_cwp
				else:
					other_wp_idx = closest_wp_idx + 1
					other_cwp = next_cwp

			# create a push the new wawypoint in the original list
			closest_wp = point_list[closest_wp_idx]
			if other_wp_idx < closest_wp_idx:
				mid_wp = (closest_wp + other_cwp) / 2
				point_list = np.insert(point_list, other_wp_idx, mid_wp, axis=0)
			else:
				mid_wp = (closest_wp + other_cwp) / 2
				point_list = np.insert(point_list, closest_wp_idx, mid_wp, axis=0)
			print("intermediate list:", len(point_list))

			(BC_x, BC_y) = bezierCurveTesting1(point_list, animate=False, write_path=None)

			if len(point_list) == 30:
				break

			idx = -1

		idx += 1

	print("new list:", len(point_list))
	visualizeNewPath(BC_x, BC_y, point_list)


def addMidPointsToPath(path):
	idx = 0
	while idx < len(path) - 1:
		mid = (path[idx] + path[idx + 1]) / 2
		path = np.insert(path, idx + 1, mid, axis=0)

		idx += 2

	return path


def main():
	point_list = np.load('final_rrt_prune_path_coords.npy')
	print("original list:", len(point_list))

	(BC_x, BC_y) = bezierCurveTesting1(point_list, animate=False, write_path=None)
	plt.ion()
	itr = 0
	visualizeNewPath(BC_x, BC_y, point_list, animate=True, write_path='./rrt_smooth_no_obs_frames', itr=itr)
	itr += 1

	# for bcx, bcy in zip(BC_x, BC_y):
	while True:
		idx = 0
		while idx < len(BC_x):
			bcx, bcy = BC_x[idx], BC_y[idx]

			# if the current waypoint is within the obstacle space => make a new waypoint
			if obs.withinObstacleSpace(point=(bcx, bcy), radius=mn.DRONE_RADIUS, clearance=(mn.DRONE_RADIUS / 2)):
				print("within obstacle")
				break
			idx += 1

		if idx == len(BC_x):
			np.save(file='final_rrt_prune_smooth_path_coords.npy', arr=point_list)
			break

		point_list = addMidPointsToPath(point_list)
		(BC_x, BC_y) = bezierCurveTesting1(point_list, animate=False, write_path=None)
		visualizeNewPath(BC_x, BC_y, point_list, animate=True, write_path='./rrt_smooth_no_obs_frames', itr=itr)
		itr += 1
	plt.ioff()
	plt.show()


if __name__ == '__main__':
	main()
