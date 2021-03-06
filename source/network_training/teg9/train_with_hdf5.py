from kzpy3.vis import *
import kzpy3.teg9.data.utils.get_data_with_hdf5 as get_data_with_hdf5
import caffe

REPO = 'kzpy3'
TEG = 'teg9'
CAF = 'caf8'
DISPLAY = True

ignore = ['reject_run','left','out1_in2','Smyth','racing'] # runs with these labels are ignored
require_one = [] # at least one of this type of run lable is required
use_states = [1]
rate_timer_interval = 5.
print_timer = Timer(5)

if True:
	MODEL = 'z2_color'
	print(MODEL)
	bair_car_data_path = opjD('bair_car_data_new_28April2017')#opjD('bair_car_data_Main_Dataset') # '/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017'#opjD('bair_car_data_new')
	#weights_file_path =  most_recent_file_in_folder(opjD(fname(opjh(REPO,CAF,MODEL))))
	weights_file_path = opjh('caffe_models/z2_color.caffemodel')
	weights_file_path = '/home/karlzipser/Desktop/z2_color_aruco/z2_color_aruco_iter_2100000.caffemodel'
	N_FRAMES = 2 # how many timesteps with images.
	N_STEPS = 10 # how many timestamps with non-image data
	gpu = 1

if False:
	MODEL = 'z2_color_small_ip1'
	print(MODEL)
	bair_car_data_path = opjD('bair_car_data_Main_Dataset') # '/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017'#opjD('bair_car_data_new')
	weights_file_path =  most_recent_file_in_folder(opjD(fname(opjh(REPO,CAF,MODEL))),['caffemodel'])
	N_FRAMES = 2 # how many timesteps with images.
	N_STEPS = 10 # how many timestamps with non-image data
	gpu = 1


if False:
	MODEL = 'z3_color'
	print(MODEL)
	bair_car_data_path = opjD('bair_car_data_Main_Dataset') # '/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017'#opjD('bair_car_data_new')
	weights_file_path = most_recent_file_in_folder(opjD(fname(opjh(REPO,CAF,MODEL))),['caffemodel'])
	#weights_file_path = opj('caffe_models/z3_color_iter_14600000.caffemodel')
	N_FRAMES = 3 # how many timesteps with images.
	N_STEPS = 30 # how many timestamps with non-image data
	gpu = 1


if False:
	MODEL = 'z1_color'
	print(MODEL)
	bair_car_data_path = opjD('bair_car_data_Main_Dataset') # '/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017'#opjD('bair_car_data_new')
	weights_file_path = most_recent_file_in_folder(opjD(fname(opjh(REPO,CAF,MODEL))),['caffemodel'])
	N_FRAMES = 1 # how many timesteps with images.
	N_STEPS = 10 # how many timestamps with non-image data
	gpu = 1








if gpu >= 0:
	caffe.set_device(gpu)
	caffe.set_mode_gpu()

import_str = "import REPO.CAF.MODEL.solver as Solver"
import_str = import_str.replace("REPO",REPO)
import_str = import_str.replace("CAF",CAF)
import_str = import_str.replace("MODEL",MODEL)
exec(import_str)

if weights_file_path:
	print(d2s("Copying weights from",weights_file_path,"to",Solver.solver))
	Solver.solver.net.copy_from(weights_file_path)
else:
	print(d2s("No weights loaded to",Solver.solver))
time.sleep(5)

hdf5_runs_path = opj(bair_car_data_path,'hdf5/runs')
hdf5_segment_metadata_path = opj(bair_car_data_path,'hdf5/segment_metadata')


loss10000 = []
loss = []

rate_timer = Timer(rate_timer_interval)
rate_ctr = 0

get_data_with_hdf5.load_Segment_Data(hdf5_segment_metadata_path,hdf5_runs_path)



print('\nloading low_steer... (takes awhile)')
low_steer = load_obj(opj(hdf5_segment_metadata_path,'low_steer'))
print('\nloading high_steer... (takes awhile)')
high_steer = load_obj(opj(hdf5_segment_metadata_path,'high_steer'))
len_high_steer = len(high_steer)
len_low_steer = len(low_steer)

ctr_low = -1 # These counter keep track of position in segment lists, and when to reshuffle.
ctr_high = -1






def get_data_considering_high_low_steer():
	global ctr_low
	global ctr_high
	global low_steer
	global high_steer

	if ctr_low >= len_low_steer:
		ctr_low = -1
	if ctr_high >= len_high_steer:
		ctr_high = -1
	if ctr_low == -1:
		random.shuffle(low_steer) # shuffle data before using (again)
		ctr_low = 0
	if ctr_high == -1:
		random.shuffle(high_steer)
		ctr_high = 0
		
	if random.random() < 0.5: # len_high_steer/(len_low_steer+len_high_steer+0.0): # with some probability choose a low_steer element
		choice = low_steer[ctr_low]
		ctr_low += 1
	else:
		choice = high_steer[ctr_high]
		ctr_high += 1
	run_code = choice[3]
	seg_num = choice[0]
	offset = choice[1]

	data = get_data_with_hdf5.get_data(run_code,seg_num,offset,N_STEPS,offset+0,N_FRAMES,ignore=ignore,require_one=require_one,use_states=use_states)

	return data



def array_to_int_list(a):
	l = []
	for d in a:
		l.append(int(d*100))
	return l



if DISPLAY:
	figure('steer',figsize=(3,2))
	figure('loss',figsize=(3,2))
	figure('high low steer histograms',figsize=(2,1))
	clf()
	plt.hist(array(low_steer)[:,2],bins=range(0,100))
	plt.hist(array(high_steer)[:,2],bins=range(0,100))
	figure(1)

while True:

	for b in range(Solver.batch_size):
		data = None
		while data == None:
			data = get_data_considering_high_low_steer()
		Solver.put_data_into_model(data,Solver.solver,b)

	Solver.solver.step(1)
	if not DISPLAY:
		if print_timer.check():
			print(Solver.solver.net.blobs['metadata'].data[-1,:,5,5])
			print(array_to_int_list(Solver.solver.net.blobs['steer_motor_target_data'].data[-1,:][:]))
			print(array_to_int_list(Solver.solver.net.blobs['ip2'].data[-1,:][:]))
			print_timer.reset()

	if DISPLAY:
		# The training step. Everything below is for display.
		rate_ctr += 1
		if rate_timer.check():
			print(d2s('rate =',dp(rate_ctr/rate_timer_interval,2),'Hz'))
			rate_timer.reset()
			rate_ctr = 0
		a = Solver.solver.net.blobs['steer_motor_target_data'].data[0,:] - Solver.solver.net.blobs['ip2'].data[0,:]
		loss.append(np.sqrt(a * a).mean())
		if len(loss) >= 10000/Solver.batch_size:
			loss10000.append(array(loss[-10000:]).mean())
			loss = []
			figure('loss');clf()
			lm = min(len(loss10000),100)
			plot(loss10000[-lm:])
			print(d2s('loss10000 =',loss10000[-1]))
		if print_timer.check():

			print(Solver.solver.net.blobs['metadata'].data[-1,:,5,5])

			cprint(array_to_int_list(Solver.solver.net.blobs['steer_motor_target_data'].data[-1,:][:]),'green','on_red')
			cprint(array_to_int_list(Solver.solver.net.blobs['ip2'].data[-1,:][:]),'red','on_green')
			
			figure('steer')
			clf()
			
			t = Solver.solver.net.blobs['steer_motor_target_data'].data[-1,:]
			o = Solver.solver.net.blobs['ip2'].data[-1,:]
			ylim(-0.05,1.05);xlim(0,len(t))
			plot([-1,60],[0.49,0.49],'k');plot(o,'og'); plot(t,'or'); plt.title(data['name'])
			
			#print(shape(Solver.solver.net.blobs['steer_motor_target_data'].data))
			#print Solver.solver.net.blobs['steer_motor_target_data'].data[-1,:]
			#print Solver.solver.net.blobs['ip2'].data[-1,:]

			mi_or_cv2_animate(data['left'],delay=33)
			pause(0.001)
			print_timer.reset()





