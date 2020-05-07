import numpy as np 
import copy
import sys
import matplotlib.pyplot as plt 

import node
import obstacles

pruned_path = []




def prunedPath(path):
	path_length = len(path)
	j = path_length -1

	# Add the last node
	pruned_path.append(path[j])
	# print(pruned_path)
	# print(pruned_path[0].current_coords)
	print("Path:")
	print(path)
	# for j in range(0,)
	while (j!= 0):
		i = 0
		for i in range(0,j):
			# print(path[i].current_coords)
			# x1, y1 = path[i].current_coords
			# x2, y2 = path[j-1].current_coords
			x1, y1 = path[j]
			x2, y2 = path[i]
			print("x1, y1: ", x1, y1)
			print("x2, y2: ", x2, y2)
			status = isObstacleLine(np.array([x2, y2]), np.array([x1, y1]))
			print("status: ", status)
			if (status ==  False):
				print("i: ", i)
				pruned_path.append(path[i])
				print("Append :",path[i])
				print("Len :",len(pruned_path))
				# j = copy.copy(i)
				j = i
				print("j: ", j)
				break

				if j == 0:
					break
			print("!-------!")
		print("_____")

	print("pruned_path: ", pruned_path)
	print("len(pruned_path): ", len(pruned_path))

	return pruned_path


def isObstacleLine(start, end):
	pts = np.linspace(0,1,num=200)
	obs = []

	for t in pts:

		eqn = t*start + (1-t)*end
		# print(eqn)
		obs.append(obstacles.withinObstacleSpace(point=eqn, radius=0, clearance=0))
	
	obs=np.array(obs)
	if np.any(obs):
		return True
	else:
		return False




def testMain():
	fig, ax = plt.subplots()
	obstacles.generateMap(ax)
	ax.set_xlim(-6, 6)
	ax.set_ylim(-6, 6)
	fig.gca().set_aspect('equal', adjustable='box')
	# path = np.load('./path.npy', allow_pickle=True)
	# print(len(path))
	# path_new = [[-4,-4],[-2, -4], [0,-4], [0, -3]]
	path_new =  np.array(
 [[-4.         ,-4.        ],
 [-3.79264131 ,-3.6579439 ],
 [-3.46906898 ,-3.42278236],
 [-3.48912813 ,-3.02328564],
 [-3.3948723  ,-2.63454943],
 [-3.1296557  ,-2.33511637],
 [-2.89075655 ,-2.01429369],
 [-2.7065285  ,-1.65924436],
 [-2.36537512 ,-1.45040382],
 [-2.07738126 ,-1.17280782],
 [-1.83184281 ,-0.85703754],
 [-2.13718564 ,-0.59864683],
 [-2.25697259 ,-0.21700421],
 [-2.17639411 , 0.17479562],
 [-1.89260448 , 0.45668824],
 [-1.71137551 , 0.81327772],
 [-1.40609844 , 1.07174612],
 [-1.00910694 , 1.12071291],
 [-0.63795093 , 1.26985452],
 [-0.30719887 , 1.49480579],
 [-0.35420504 , 1.89203421],
 [ 0.02984229 , 2.0038718 ],
 [ 0.42573924 , 1.94672632],
 [ 0.47830175 , 1.55019488],
 [ 0.6912239  , 1.21157391],
 [ 1.07448352 , 1.097066  ],
 [ 1.47007429 , 1.03783814],
 [ 1.86993423 , 1.02725365],
 [ 2.22281226 , 1.2156073 ],
 [ 2.43382091 , 1.55542393],
 [ 2.76428396 , 1.3300483 ],
 [ 3.15622053 , 1.40995904],
 [ 3.22079987 , 1.80471151],
 [ 3.27615393 , 2.2008629 ],
 [ 3.37145145 , 2.58934505],
 [ 3.63297828 , 2.89200612],
 [ 3.7511519  , 3.27415137],
 [ 3.50783803 , 3.59163894],
 [ 4.         , 4.        ]])

	# path_new = [[-4,-3],[-3,-4], [-2,-4.5], [-0.5,-4], [0,-2],[2,0]]
	pr_path=prunedPath(path=path_new)
	pr_path= np.array(pr_path)
	plt.plot(path_new[:,0],path_new[:,1],'g')
	plt.plot(pr_path[:,0],pr_path[:,1],'r')
	
	plt.show()
	# val = isObstacleLine(np.array([-0.5,-3]), np.array([-2,-4.2]))
	# print(val)
	# sys.exit(0)



if __name__ == '__main__':
	testMain()