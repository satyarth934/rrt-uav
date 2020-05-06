from __future__ import print_function, division
# import os
import sys
import numpy as np
import matplotlib.pyplot as plt


def plotPoints(points_list, subplot_ax):
	for point in points_list:
		dot = plt.Circle(point, radius=0.2, color='black')
		subplot_ax.add_artist(dot)
