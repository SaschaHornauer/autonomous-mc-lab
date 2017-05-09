"""
Create prototxt strings
"""

REPO = 'kzpy3'
CAF = 'caf8'
MODEL = 'zn_color'
exec('from '+REPO+'.utils import *')

import caffe

def conv(top,bottom,num_output,group,kernel_size,stride,pad,weight_filler_type,std=0):
	if type(pad) == int:
		pad_h = pad
		pad_w = pad
	else:
		pad_h = pad[0]
		pad_w = pad[1]
	p = """
layer {
\tname: "TOP"
\ttype: "Convolution"
\tbottom: "BOTTOM"
\ttop: "TOP"
\tconvolution_param {
\t\tnum_output: NUM_OUTPUT
\t\tgroup: NUM_GROUP
\t\tkernel_size: KERNEL_SIZE
\t\tstride: STRIDE
\t\tpad_h: PAD_H
\t\tpad_w: PAD_W
\t\tweight_filler {
\t\t\ttype: "WEIGHT_FILLER_TYPE" STD
\t\t}
\t}
}
	"""
	p = p.replace("TOP",top)
	p = p.replace("BOTTOM",bottom)
	p = p.replace("NUM_OUTPUT",str(num_output))
	p = p.replace("NUM_GROUP",str(group))
	p = p.replace("KERNEL_SIZE",str(kernel_size))
	p = p.replace("STRIDE",str(stride))
	p = p.replace("PAD_H",str(pad_h))
	p = p.replace("PAD_W",str(pad_w))
	p = p.replace("WEIGHT_FILLER_TYPE",weight_filler_type)
	if weight_filler_type == 'gaussian':
		p = p.replace("STD","\n\t\t\tstd: "+str(std))
	else:
		p = p.replace("STD","")
	return p

def relu(bottom):
	p = """
layer {
\tname: "BOTTOM_relu"
\ttype: "ReLU"
\tbottom: "BOTTOM"
\ttop: "BOTTOM"
}
	"""
	p = p.replace("BOTTOM",bottom)
	return p

def drop(bottom,ratio):
	p = """
layer {
\tname: "BOTTOM_drop"
\ttype: "Dropout"
\tbottom: "BOTTOM"
\ttop: "BOTTOM"
\tdropout_param {
\t\tdropout_ratio: RATIO
\t}
}
	"""
	p = p.replace("BOTTOM",bottom)
	p = p.replace("RATIO",str(ratio))
	return p

def deconv(top,bottom,num_output,group,kernel_size,stride,pad,weight_filler_type,std=0):
	return conv(top,bottom,num_output,group,kernel_size,stride,pad,weight_filler_type,std).replace('Convolution','Deconvolution')

def pool(bottom,p_type,kernel_size,stride,pad=0):
	if type(pad) == int:
		pad_h = pad
		pad_w = pad
	else:
		pad_h = pad[0]
		pad_w = pad[1]
	p = """
layer {
\tname: "BOTTOM_pool"
\ttype: "Pooling"
\tbottom: "BOTTOM"
\ttop: "BOTTOM_pool"
\tpooling_param {
\t\tpool: POOL_TYPE
\t\tkernel_size: KERNEL_SIZE
\t\tstride: STRIDE
\t\tpad_h: PAD_H
\t\tpad_w: PAD_W
\t}
}
	"""
	p = p.replace("BOTTOM",bottom)
	p = p.replace("POOL_TYPE",p_type)
	p = p.replace("KERNEL_SIZE",str(kernel_size))
	p = p.replace("STRIDE",str(stride))
	p = p.replace("PAD_H",str(pad_h))
	p = p.replace("PAD_W",str(pad_w))
	return p

def conv_layer_set(
	c_top,
	c_bottom,
	c_num_output,
	c_group,
	c_kernel_size,
	c_stride,
	c_pad,
	p_type,
	p_kernel_size,
	p_stride,
	p_pad,
	weight_filler_type,std=0):
	p = """\n###################### Convolutional Layer Set '"""+c_top+"""' ######################\n#"""
	p = p + conv(c_top,c_bottom,c_num_output,c_group,c_kernel_size,c_stride,c_pad,weight_filler_type,std)
	p = p + relu(c_top)
	p = p + pool(c_top,p_type,p_kernel_size,p_stride,p_pad)
	p = p + '\n#\n############################################################\n\n'
	return p

def dummy(top,dims):
	p = """
layer {
\tname: "TOP"
\ttype: "DummyData"
\ttop: "TOP"
\tdummy_data_param {
\t\tshape {
DIMS
\t\t}
\t}
}
	"""
	p = p.replace('TOP',top)
	d = ""
	for i in range(len(dims)):
		d = d + d2s('\t\t\tdim:',dims[i])
		if i < len(dims)-1:
			d = d + '\n'
	p = p.replace('DIMS',d)
	return p

def python(top,bottom,module,layer,phase=False):
	p = """
layer {
\ttype: 'Python'
\tname: 'TOP'
\tbottom: 'BOTTOM'
\ttop: 'TOP'
\tpython_param {
\t\tmodule: 'MODULE'
\t\tlayer: 'LAYER'
\t}
		"""
	if phase:
		p = p + """
\tinclude {
\t\tphase: PHASE
\t}
"""
	p = p + "\n}"
	p = p.replace('TOP',top)
	p = p.replace('BOTTOM',bottom)
	p = p.replace('MODULE',module)
	p = p.replace('LAYER',layer)
	if phase:
		p = p.replace('PHASE',phase)
	return p

def concat(top,bottom_list,axis):
	p = """
layer {
\ttype: 'Concat'
\tname: 'TOP'
BOTTOM_LIST
\ttop: 'TOP'
\tconcat_param {
\t\taxis: AXIS
\t}
}
		"""
	bottom_list_str = ""
	for i in range(len(bottom_list)):
		b = bottom_list[i]
		bottom_list_str += """\tbottom: \""""+b+"""\""""
		if i < len(bottom_list) - 1:
			bottom_list_str += '\n'
	p = p.replace('TOP',top)
	p = p.replace('BOTTOM_LIST',bottom_list_str)
	p = p.replace('AXIS',str(axis))

	return p
def ip(top,bottom,num_output,weight_filler_type,std=0):
	p = """
layer {
\tname: "TOP"
\ttype: "InnerProduct"
\tbottom: "BOTTOM"
\ttop: "TOP"
\tinner_product_param {
\t\tnum_output: NUM_OUTPUT
\t\tweight_filler {
\t\t\ttype: "WEIGHT_FILLER_TYPE" STD
\t\t}
\t}
}
	"""
	p = p.replace("TOP",top)
	p = p.replace("BOTTOM",bottom)
	p = p.replace("NUM_OUTPUT",str(num_output))
	p = p.replace("WEIGHT_FILLER_TYPE",weight_filler_type)
	if weight_filler_type == 'gaussian':
		p = p.replace("STD","\n\t\t\tstd "+str(std))
	else:
		p = p.replace("STD","")
	return p

def ip_layer_set(top,bottom,num_output,weight_filler_type,std=0):
	p = """\n###################### IP Layer Set '"""+top+"""' ######################\n#"""
	p = p + ip(top,bottom,num_output,weight_filler_type,std)
	p = p + relu(top)
	p = p + '\n#\n############################################################\n\n'
	return p

def euclidean(top,bottom1,bottom2):
	p = """
layer {
\tname: "TOP"
\ttype: "EuclideanLoss"
\tbottom: "BOTTOM1"
\tbottom: "BOTTOM2"
\ttop: "TOP"
\tloss_weight: 1
}
	"""
	p = p.replace("TOP",top)
	p = p.replace("BOTTOM1",bottom1)
	p = p.replace("BOTTOM2",bottom2)
	return p

def scale(top,bottom,scale,bias):
	p = """
layer {
  name: "TOP"
  type: "Scale"
  bottom: "BOTTOM"
  top: "TOP"
  param {
    lr_mult: 0
    decay_mult: 0
  }
  param {
    lr_mult: 0
    decay_mult: 0
  }
  scale_param {
    filler {
      value: SCALE    }
    bias_term: true
    bias_filler {
      value: BIAS
    }
  }
}
	"""
	p = p.replace("TOP",top)
	p = p.replace("BOTTOM",bottom)
	p = p.replace("SCALE",str(scale))
	p = p.replace("BIAS",str(bias))
	return p

def solver_proto(
	model_path='kzpy3/caf6/z2_color_streamline',
	test_iter=1,
	test_interval=1000000,
	test_initialization='false',
	base_lr = 0.01,
	momentum=0.0001,
	weight_decay='0.000005',
	lr_policy="inv",
	gamma=0.0001,
	power=0.75,
	display=20000,
	max_iter=10000000,
	snapshot=100000
	):

	solver_str = """
net: \""""+opj(model_path,"train_val.prototxt")+"""\"
test_iter: """+d2n(test_iter)+"""
test_interval: """+d2n(test_interval)+"""
test_initialization: """+d2n(test_initialization)+"""
base_lr: """+d2n(base_lr)+"""  
momentum: """+d2n(momentum)+"""
weight_decay: """+d2n(weight_decay)+"""
lr_policy: \""""+d2n(lr_policy)+"""\"
gamma: """+d2n(gamma)+"""
power: """+d2n(power)+"""
display: """+d2n(display)+"""
max_iter: """+d2n(max_iter)+"""
snapshot: """+d2n(snapshot)+"""
snapshot_prefix: \""""+opjD(fname(model_path),fname(model_path))+"""\"
	"""
	#print solver_str
	#list_of_strings_to_txt_file(opj(model_path,'solver.prototxt'),[solver_str])
	return solver_str












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
