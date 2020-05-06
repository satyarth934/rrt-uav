from __future__ import print_function, division
# import os
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

import utils


def bezierCurveEquationUtil(P0, P1, P2, P3, axis_idx, step_size=0.1):
	t = np.arange(0, (1 + step_size), step_size)
	P0_list = [P0[axis_idx]] * (1 + int(1 / step_size))
	P1_list = [P1[axis_idx]] * (1 + int(1 / step_size))
	P2_list = [P2[axis_idx]] * (1 + int(1 / step_size))
	P3_list = [P3[axis_idx]] * (1 + int(1 / step_size))

	Bt = (((1 - t)**3) * P0_list) + (3 * ((1 - t)**2) * t * P1_list) + (3 * (1 - t) * (t**2) * P2_list) + ((t**3) * P3_list)

	return Bt


def bezierCurveEquation(P0, P1, P2, P3, step_size=0.1):
	bc_x = bezierCurveEquationUtil(P0, P1, P2, P3, axis_idx=0, step_size=step_size)
	bc_y = bezierCurveEquationUtil(P0, P1, P2, P3, axis_idx=1, step_size=step_size)

	return (bc_x, bc_y)


def bezierCurveTesting0():
	P0 = (1, 1)
	P1 = (10, 20)
	P2 = (15, 18)
	P3 = (17, 8)

	bc_x, bc_y = bezierCurveEquation(P0, P1, P2, P3, step_size=0.01)

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

	plt.plot(bc_x, bc_y, color='green')
	plt.show()


def bezierCurveTesting1():
	point_list = np.random.randint(0, 25, (15, 2))
	print(point_list)

	BC_x = []
	BC_y = []

	P0 = point_list[0]
	P1 = point_list[1]
	P2 = point_list[2]
	P3 = point_list[3]

	bc_x, bc_y = bezierCurveEquation(P0, P1, P2, ((P2 + P3) / 2), step_size=0.01)
	BC_x.extend(bc_x)
	BC_y.extend(bc_y)

	point_idx = 4
	while point_idx < len(point_list):
		P0 = P2
		P1 = P3
		P2 = point_list[point_idx]
		point_idx += 1
		try:
			P3 = point_list[point_idx]
			point_idx += 1

			bc_x, bc_y = bezierCurveEquation(((P0 + P1) / 2), P1, P2, ((P2 + P3) / 2), 0.01)
			BC_x.extend(bc_x)
			BC_y.extend(bc_y)
		except Exception as e:
			print("ERROR =>>>", e)

	fig, ax = plt.subplots()
	ax.set_xlim(-1, 25)
	ax.set_ylim(-1, 25)

	utils.plotPoints(point_list, ax)

	plt.plot(BC_x, BC_y, color='green')
	plt.show()


def main():
	bezierCurveTesting1()


if __name__ == '__main__':
	main()
