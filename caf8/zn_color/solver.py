
REPO = 'kzpy3'
CAF = 'caf8'
MODEL = 'zn_color'
exec('from '+REPO+'.utils import *')
exec('import '+REPO+'.'+CAF+'.protos as protos')



##############################################################################
#
model_path = opjh(REPO,CAF,MODEL)


train_val_lst = [d2s('#',model_path),d2s('#',time_str('Pretty'))]

train_val_lst += [
	d2s('#',model_path),
	d2s('#',time_str('Pretty')),

	protos.dummy('steer_motor_target_data',(1,20)),
	protos.dummy('metadata',(1,6,14,26)),
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

	protos.ip("ip2","ip1",20,"xavier",std=0),
	protos.euclidean("euclidean","steer_motor_target_data","ip2")
]


solver_lst =  [
	d2s('#',model_path),
	d2s('#',time_str('Pretty')),
	protos.solver_proto(
	model_path=model_path,
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
list_of_strings_to_txt_file(opj(model_path,'solver.prototxt'),solver_lst)





def put_data_into_model(data,solver):
	############## load data into solver #####################
	#
	ctr = 0
	for c in range(3):
		for camera in ('left','right'):
			for t in range(N_FRAMES):
				solver.net.blobs['ZED_data_pool2'].data[0,ctr,:,:] = data[camera][t][:,:,c]
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
	solver.net.blobs['metadata'].data[0,0,:,:] = Racing
	solver.net.blobs['metadata'].data[0,1,:,:] = Caf
	solver.net.blobs['metadata'].data[0,2,:,:] = Follow
	solver.net.blobs['metadata'].data[0,3,:,:] = Direct
	solver.net.blobs['metadata'].data[0,4,:,:] = Play
	solver.net.blobs['metadata'].data[0,5,:,:] = Furtive
	solver.net.blobs['steer_motor_target_data'].data[0,:N_STEPS] = data['steer'][-N_STEPS:]/99.
	solver.net.blobs['steer_motor_target_data'].data[0,N_STEPS:] = data['motor'][-N_STEPS:]/99.
	#
	##########################################################










