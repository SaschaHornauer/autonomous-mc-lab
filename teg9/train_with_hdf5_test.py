from kzpy3.utils import *
import caffe

def print_solver(solver):
	print("")
	for l in [(k, v[0].data.shape) for k, v in solver.net.params.items()]:
		print(l)
	print("")
	for l in [(k, v.data.shape) for k, v in solver.net.blobs.items()]:
		if 'split' not in l[0]:
			print(l)

def setup_solver(solver_file_path):
	solver = caffe.SGDSolver(solver_file_path)
	print_solver(solver)
	return solver



if True:
	solver = setup_solver('/Users/karlzipser/kzpy3/caf8/test/solver.prototxt')
	#weights_file_path = opjh('kzpy3/caf5/z2_color/z2_color.caffemodel')
	#weights_file_path = opjD('z2_color_aruco_boundary_1st_pass/z2_color_iter_22600000.caffemodel')
	#weights_file_path = opjD('z2_color/z2_color_iter_100000.caffemodel')
	#weights_file_path = '/home/karlzipser/Desktop/z2_color_aruco3/z2_color_iter_11900000.caffemodel'
	#weights_file_path = '/home/karlzipser/Desktop/z2_color/z2_color_iter_100000.caffemodel'
	#solver.net.copy_from(weights_file_path)
	#cprint('Loaded weights from '+weights_file_path)
	
