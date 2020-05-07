from __future__ import print_function, division
# import os
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

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


def bezierCurveTesting1(animate=False, write=False):
	# point_list = np.random.randint(0, 100, (10, 2))
	# print(point_list)
	# point_list.sort(axis=0)
	# print(point_list)
	point_list = np.load('../Results/rrt_path_coords.npy')

	fig, ax = plt.subplots()
	fig.gca().set_aspect('equal', adjustable='box')
	ax.set_xlim(-6, 6)
	ax.set_ylim(-6, 6)

	obs.generateMap(plotter=ax)

	utils.plotPoints(point_list, ax, radius=0.04, color='black')
	utils.plotPoint(point_list[0], ax, radius=0.06, color='cyan') 		# start
	utils.plotPoint(point_list[-1], ax, radius=0.06, color='magenta') 	# end

	if write:
		write_itr = 0
		plt.savefig('../Results/rrt_smooth_frames/%s.png' % (str(write_itr)))
		write_itr += 1

	if animate:
		plt.ion()

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

	# Animation
	if animate:
		utils.plotPoints([P0, P1, P2, P3], ax, radius=0.04, color='red')
		plt.plot(bc_x, bc_y, color='green')
		plt.show()
		plt.pause(1)

	if write:
		plt.savefig('../Results/rrt_smooth_frames/%s.png' % (str(write_itr)))
		write_itr += 1

	if animate:
		utils.plotPoints([P0, P1, P2, P3], ax, radius=0.04, color='yellow')
	##############

	# Handling the rest of the points, leaving out the last 1 or 2 points for the end
	point_idx = 4
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

		# Animation
		if animate:
			utils.plotPoints([P0, P1, P2, P3], ax, radius=0.04, color='red')
			plt.plot(bc_x, bc_y, color='green')
			plt.show()
			plt.pause(1)

		if write:
			plt.savefig('../Results/rrt_smooth_frames/%s.png' % (str(write_itr)))
			write_itr += 1

		if animate:
			utils.plotPoints([P0, P1, P2, P3], ax, radius=0.04, color='yellow')
		##############

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

	# Animation
	if animate:
		utils.plotPoints([P0, P1, P2, P3], ax, radius=0.04, color='red')
		plt.plot(bc_x, bc_y, color='green')
		plt.show()
		plt.pause(1)

	if write:
		plt.savefig('../Results/rrt_smooth_frames/%s.png' % (str(write_itr)))
		write_itr += 1

	if animate:
		utils.plotPoints([P0, P1, P2, P3], ax, radius=0.04, color='yellow')
	##############

	# Animation
	if animate:
		plt.ioff()
	else:
		plt.plot(BC_x, BC_y, color='green')

	plt.show()

	if write:
		plt.savefig('../Results/rrt_smooth_frames/%s.png' % (str(write_itr)))
	##############


def main():
	bezierCurveTesting1(animate=True, write=True)


if __name__ == '__main__':
	main()
