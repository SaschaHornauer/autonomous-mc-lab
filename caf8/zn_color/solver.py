
REPO = 'kzpy3'
CAF = 'caf8'
MODEL = 'zn_color'
#exec('from '+REPO+'.utils import *')
#exec('import '+REPO+'.'+CAF+'.protos as protos')

from kzpy3.utils import *
import kzpy3.caf8.protos as protos

##############################################################################
#
model_path = opjh(REPO,CAF,MODEL)

batch_size = 256

"""
('steer_motor_target_data', (1, 40))
('metadata', (1, 10, 14, 26))
('color_data', (1, 12, 376, 672))
('motion_data', (1, 10, 376, 672))
('color_data_pool', (1, 12, 188, 336))
('color_data_pool_pool', (1, 12, 94, 168))
('motion_data_pool', (1, 10, 188, 336))
('motion_data_pool_pool', (1, 10, 94, 168))
('motion_data_pool_pool_pool', (1, 10, 47, 84))
('conv1m', (1, 96, 14, 26))
('conv1', (1, 96, 28, 53))
('conv1_pool', (1, 96, 14, 26))
('conv1_metadata_concat', (1, 202, 14, 26))
('conv2', (1, 256, 6, 12))
('conv2_pool', (1, 256, 3, 6))
('ip1', (1, 512))
('ip2', (1, 40))
('euclidean', ())
"""


train_val_lst = [d2s('#',model_path),d2s('#',time_str('Pretty'))]

train_val_lst += [
	d2s('#',model_path),
	d2s('#',time_str('Pretty')),

	protos.dummy('steer_motor_target_data',(batch_size,40)),
	protos.dummy('metadata',(batch_size,10,14,26)),
	protos.dummy('color_data_pool_pool',(batch_size,12,94,168)),
	protos.dummy('motion_data_pool_pool',(batch_size,10,94,168)),
	protos.scale('color_data_pool_pool_scale','color_data_pool_pool',0.003921,-0.5),
	protos.scale('motion_data_pool_pool_scale','motion_data_pool_pool',0.003921,-0.5),

	protos.pool("motion_data_pool_pool_scale","AVE",3,2,0),

	protos.conv("conv1m",'motion_data_pool_pool_scale_pool',96,1,11,3,[2,1],"gaussian",std='0.00001'),
	protos.relu("conv1m"),

	protos.conv("conv1",'color_data_pool_pool_scale',96,1,11,3,0,"gaussian",std='0.00001'),
	protos.relu("conv1"),
	protos.pool("conv1","MAX",3,2,0),

	protos.concat('conv1_metadata_concat',["conv1m","conv1_pool","metadata"],1), 

	protos.conv("conv2",'conv1_metadata_concat',256,2,3,2,0,"gaussian",std='0.1'),
	protos.relu("conv2"),
	protos.pool("conv2","MAX",3,2,0),
	protos.drop('conv2_pool',0.0),
	protos.ip("ip1","conv2_pool",512,"xavier",std=0),
	protos.relu('ip1'),

	protos.ip("ip2","ip1",40,"xavier",std=0),
	protos.euclidean("euclidean","steer_motor_target_data","ip2")
]
"""
train_val_lst += [
	d2s('#',model_path),
	d2s('#',time_str('Pretty')),

	protos.dummy('steer_motor_target_data',(1,40)),
	protos.dummy('metadata',(1,10,14,26)),
	protos.dummy('color_data',(1,12,376,672)),
	protos.dummy('motion_data',(1,10,376,672)),

	protos.pool("color_data","AVE",3,2,0),
	protos.pool("color_data_pool","AVE",3,2,0),

	protos.pool("motion_data","AVE",3,2,0),
	protos.pool("motion_data_pool","AVE",3,2,0),
	protos.pool("motion_data_pool_pool","AVE",3,2,0),

	protos.conv("conv1m",'motion_data_pool_pool_pool',96,1,11,3,[2,1],"gaussian",std='0.00001'),
	protos.relu("conv1m"),

	protos.conv("conv1",'color_data_pool_pool',96,1,11,3,0,"gaussian",std='0.00001'),
	protos.relu("conv1"),
	protos.pool("conv1","MAX",3,2,0),

	protos.concat('conv1_metadata_concat',["conv1m","conv1_pool","metadata"],1), 

	protos.conv("conv2",'conv1_metadata_concat',256,2,3,2,0,"gaussian",std='0.1'),
	protos.relu("conv2"),
	protos.pool("conv2","MAX",3,2,0),
	protos.drop('conv2_pool',0.0),
	protos.ip("ip1","conv2_pool",512,"xavier",std=0),
	protos.relu('ip1'),

	protos.ip("ip2","ip1",40,"xavier",std=0),
	protos.euclidean("euclidean","steer_motor_target_data","ip2")
]
"""

solver_lst =  [
	d2s('#',model_path),
	d2s('#',time_str('Pretty')),
	protos.solver_proto(
	model_path=model_path,
	test_iter=1,
	test_interval=1000000,
	test_initialization='false',
	base_lr = 0.001,
	momentum=0.0001,
	weight_decay='0.000005',
	lr_policy="inv",
	gamma=0.0001,
	power=0.75,
	display=20000,
	max_iter=10000000,
	snapshot=100000
	)]
#
##############################################################################


unix(d2s('mkdir -p',opjD(fname(model_path))))

for t in train_val_lst:
	print t
print('')
for t in solver_lst:
	print t

list_of_strings_to_txt_file(opj(model_path,'train_val.prototxt'),train_val_lst)
solver_path = opj(model_path,'solver.prototxt')
list_of_strings_to_txt_file(solver_path,solver_lst)

solver = protos.setup_solver(solver_path)



def put_data_into_model(data,solver,b=0):
	############## load data into solver #####################
	#
	ctr = 0
	for side in ['left','right']:
		for t in range(2):
			for c in range(3):
				solver.net.blobs['color_data_pool_pool'].data[b,ctr,:,:] = data[side][t][:,:,c]
				ctr += 1
	ctr = 0
	for side in ['left']:
		for t in range(10):
			for c in range(1):
				solver.net.blobs['motion_data_pool_pool'].data[b,ctr,:,:] = data[side][t][:,:,c]
				ctr += 1
	Racing = 0
	Caf = 0
	Follow = 0
	Direct = 0
	Play = 0
	Furtive = 0
	if data['labels']['racing']:
		Racing = 1.0
	if data['states'][0] == 6:
		Caf = 1.0
	if data['labels']['follow']:
		Follow = 1.0
	if data['labels']['direct']:
		Direct = 1.0
	if data['labels']['play']:
		Play = 1.0
	if data['labels']['furtive']:
		Furtive = 1.0

	current = {}	
	current['steer'] = data['steer'][7:10].mean()
	current['motor'] = data['motor'][7:10].mean()

	solver.net.blobs['metadata'].data[b,0,:,:] = Racing
	solver.net.blobs['metadata'].data[b,1,:,:] = Caf
	solver.net.blobs['metadata'].data[b,2,:,:] = Follow
	solver.net.blobs['metadata'].data[b,3,:,:] = Direct
	solver.net.blobs['metadata'].data[b,4,:,:] = Play
	solver.net.blobs['metadata'].data[b,5,:,:] = Furtive
	solver.net.blobs['metadata'].data[b,6,:,:] = current['steer']/100.
	solver.net.blobs['metadata'].data[b,7,:,:] = current['motor']/100.	
	#solver.net.blobs['metadata'].data[b,8,:,:] = x_gradient
	#solver.net.blobs['metadata'].data[b,9,:,:] = y_gradient
	
	SM = {}
	SM['steer'] = {}
	SM['motor'] = {}
	SM['steer']['pos'] = zeros(10)
	SM['steer']['neg'] = zeros(10)
	SM['motor']['pos'] = zeros(10)
	SM['motor']['neg'] = zeros(10)

	for m in ['steer','motor']:
		ctr = 0
		for i in range(10,40,3):
			d = (data[m][i:i+3].mean()-49)/100.# current[m])/100.0
			if d > 0:
				SM[m]['pos'][ctr] = d
			else:
				SM[m]['neg'][ctr] = -d
			ctr += 1

	solver.net.blobs['steer_motor_target_data'].data[b,0:10]  = SM['steer']['pos']
	solver.net.blobs['steer_motor_target_data'].data[b,10:20] = SM['steer']['neg']
	solver.net.blobs['steer_motor_target_data'].data[b,20:30] = SM['motor']['pos']
	solver.net.blobs['steer_motor_target_data'].data[b,30:40] = SM['motor']['neg']


	#
	##########################################################










