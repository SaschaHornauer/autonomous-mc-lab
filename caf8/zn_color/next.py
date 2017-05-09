#from common import *
from kzpy3.caf8.zn_color.common import *

os.environ['GLOG_minloglevel'] = '2'

import caffe

gpu = 1
if gpu >= 0:
	caffe.set_device(gpu)
	caffe.set_mode_gpu()

exec('from '+REPO+'.'+CAF+'.'+MODEL+'.solver import solver,model_path')


weights_file_path =  most_recent_file_in_folder(opjD('z2_color_aruco'))#fname(model_path)))

if weights_file_path:
	print(d2s("Copying weights from"+weights_file_path+"to",solver))
	solver.net.copy_from(weights_file_path)
else:
	print(d2s("No weights loaded to",solver))

