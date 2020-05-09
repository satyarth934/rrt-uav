from __future__ import print_function, division
# import os
import sys
import numpy as np
# import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

# import utils


def bezierCubicCurveEquationUtil(P0, P1, P2, P3, axis_idx, step_size=0.1):
	t = np.arange(0, (1 + step_size), step_size)
	P0_list = np.array([P0[axis_idx]] * (1 + int(1 / step_size)))
	P1_list = np.array([P1[axis_idx]] * (1 + int(1 / step_size)))
	P2_list = np.array([P2[axis_idx]] * (1 + int(1 / step_size)))
	P3_list = np.array([P3[axis_idx]] * (1 + int(1 / step_size)))

	Bt = (((1 - t)**3) * P0_list) + (3 * ((1 - t)**2) * t * P1_list) + (3 * (1 - t) * (t**2) * P2_list) + ((t**3) * P3_list)

	return Bt


def bezierCubicCurveEquation(P0, P1, P2, P3, step_size=0.1):
	bc_x = bezierCubicCurveEquationUtil(P0, P1, P2, P3, axis_idx=0, step_size=step_size)
	bc_y = bezierCubicCurveEquationUtil(P0, P1, P2, P3, axis_idx=1, step_size=step_size)

	return (bc_x, bc_y)


def bezierQuadraticCurveEquationUtil(P0, P1, P2, axis_idx, step_size=0.1):
	t = np.arange(0, (1 + step_size), step_size)
	P0_list = np.array([P0[axis_idx]] * (1 + int(1 / step_size)))
	P1_list = np.array([P1[axis_idx]] * (1 + int(1 / step_size)))
	P2_list = np.array([P2[axis_idx]] * (1 + int(1 / step_size)))

	Bt = P1_list + ((1 - t)**2) * (P0_list - P1_list) + (t**2) * (P2_list - P1_list)

	return Bt


def bezierQuadraticCurveEquation(P0, P1, P2, step_size=0.1):
	bc_x = bezierQuadraticCurveEquationUtil(P0, P1, P2, axis_idx=0, step_size=step_size)
	bc_y = bezierQuadraticCurveEquationUtil(P0, P1, P2, axis_idx=1, step_size=step_size)

	return (bc_x, bc_y)


def bezierLinearCurveEquationUtil(P0, P1, axis_idx, step_size=0.1):
	t = np.arange(0, (1 + step_size), step_size)
	P0_list = np.array([P0[axis_idx]] * (1 + int(1 / step_size)))
	P1_list = np.array([P1[axis_idx]] * (1 + int(1 / step_size)))

	Bt = (1 - t) * P0_list + t * P1_list

	return Bt


def bezierLinearCurveEquation(P0, P1, step_size=0.1):
	bc_x = bezierLinearCurveEquationUtil(P0, P1, axis_idx=0, step_size=step_size)
	bc_y = bezierLinearCurveEquationUtil(P0, P1, axis_idx=1, step_size=step_size)

	return (bc_x, bc_y)
