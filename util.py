import numpy as np

if __name__ == "__main__":
	positions = np.load('/home/sandeep/px4_ws/src/px4_controller/rrt-uav/Results/rrt_path_coords.npy',allow_pickle=True)
	
	f = open('pos.txt', 'w+')
	for p in positions:
		f.write(str(p) + '\n')
	f.close()
