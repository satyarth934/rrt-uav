import os
import sys


def function():
	dir_name = sys.argv[1]

	frames = [os.path.join(dir_name, frame) for frame in os.listdir(dir_name)]

	for frame in frames:
		new_frame_name = "%04d.png" % (int(os.path.basename(frame).split(".")[0]))
		new = os.path.join(dir_name, new_frame_name)
		os.rename(frame, new)


def main():
	function()


if __name__ == '__main__':
	main()