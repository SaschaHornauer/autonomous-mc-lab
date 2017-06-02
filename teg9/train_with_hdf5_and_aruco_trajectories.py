from kzpy3.vis import *
import kzpy3.teg9.data.utils.get_data_with_hdf5 as get_data_with_hdf5
import caffe

REPO = 'kzpy3'
TEG = 'teg9'
CAF = 'caf8'
DISPLAY = True

ignore = ['reject_run','left','out1_in2','Smyth','racing','local','Tilden','campus'] # runs with these labels are ignored
require_one = ['aruco_ring'] # at least one of this type of run lable is required
use_states = [1,3,5,6,7]
rate_timer_interval = 5.
print_timer = Timer(10)

if False:
	MODEL = 'z2_color'
	print(MODEL)
	bair_car_data_path = opjD('bair_car_data_new_28April2017') #opjD('bair_car_data_Main_Dataset') # opjD('bair_car_data_new')
	weights_file_path =  most_recent_file_in_folder(opjD(MODEL),['caffemodel'])
	#weights_file_path = opjh('caffe_models/z2_color.caffemodel')
	N_FRAMES = 2 # how many timesteps with images.
	N_STEPS = 10 # how many timestamps with non-image data
	gpu = 1

if True:
	MODEL = 'z2_color_aruco'
	print(MODEL)
	bair_car_data_path = opjD('bair_car_data_new_28April2017') #opjD('bair_car_data_Main_Dataset') # opjD('bair_car_data_new')
	weights_file_path =  most_recent_file_in_folder(opjD(MODEL),['caffemodel'])
	#weights_file_path = opjh('caffe_models/z2_color.caffemodel')
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
	bair_car_data_path = '/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017'
	weights_file_path = most_recent_file_in_folder(opjD(fname(opjh(REPO,CAF,MODEL))),['caffemodel'])
	N_FRAMES = 1 # how many timesteps with images.
	N_STEPS = 10 # how many timestamps with non-image data
	gpu = 1



"""
def sample_dic:
"""




if False:
	CS_('load aruco trajectory data')
	Aruco_Steering_Trajectories = {}
	aruco_data_location = opjD('output_data')
	for o in sggo(aruco_data_location,'*.output_data.pkl'):
		ast = lo(o)
		for run_name in ast.keys():	
			print(run_name)
			if run_name not in Aruco_Steering_Trajectories:
				Aruco_Steering_Trajectories[run_name] = {}
			if len(ast[run_name].keys()) != 4:
				print_stars()
				continue
			for mode in ast[run_name].keys():
				print('\t'+mode)
				if mode not in Aruco_Steering_Trajectories[run_name]:
					Aruco_Steering_Trajectories[run_name][mode] = {}
				timestamps = ast[run_name][mode]['near_t']
				steer = ast[run_name][mode]['steer']
				velocity = ast[run_name][mode]['velocity']
				other_car_inverse_distances = ast[run_name][mode]['other_car_inverse_distances']
				marker_inverse_distances = ast[run_name][mode]['marker_inverse_distances']				
				assert(len(timestamps) == len(steer))
				assert(len(timestamps) == len(velocity))
				assert(len(timestamps) == len(other_car_inverse_distances))
				assert(len(timestamps) == len(marker_inverse_distances))
				Aruco_Steering_Trajectories[run_name][mode]['new_steer'] = {}
				for t,s in zip(timestamps,steer):
					Aruco_Steering_Trajectories[run_name][mode]['new_steer'][t] = s
				Aruco_Steering_Trajectories[run_name][mode]['velocity'] = {}
				for t,s in zip(timestamps,velocity):
					Aruco_Steering_Trajectories[run_name][mode]['velocity'][t] = s
				Aruco_Steering_Trajectories[run_name][mode]['other_car_inverse_distances'] = {}
				for t,s in zip(timestamps,other_car_inverse_distances):
					Aruco_Steering_Trajectories[run_name][mode]['other_car_inverse_distances'][t] = s
				Aruco_Steering_Trajectories[run_name][mode]['marker_inverse_distances'] = {}
				for t,s in zip(timestamps,marker_inverse_distances):
					Aruco_Steering_Trajectories[run_name][mode]['marker_inverse_distances'][t] = s

	for run_name in Aruco_Steering_Trajectories.keys():
		if 'flip_' in run_name:
			del Aruco_Steering_Trajectories[run_name]
			continue
		flip = 'flip_'+run_name
		Aruco_Steering_Trajectories[flip]= {}
		for mode in Aruco_Steering_Trajectories[run_name]:
			#if mode not in Aruco_Steering_Trajectories[flip]:
			Aruco_Steering_Trajectories[flip][mode] = {}
			Aruco_Steering_Trajectories[flip][mode]['new_steer'] = {}
			Aruco_Steering_Trajectories[flip][mode]['velocity'] = {}
			Aruco_Steering_Trajectories[flip][mode]['other_car_inverse_distances'] = {}
			Aruco_Steering_Trajectories[flip][mode]['marker_inverse_distances'] = {}
			for t in Aruco_Steering_Trajectories[run_name][mode]['new_steer'].keys():
				Aruco_Steering_Trajectories[flip][mode]['new_steer'][t] = 99-Aruco_Steering_Trajectories[run_name][mode]['new_steer'][t]
				Aruco_Steering_Trajectories[flip][mode]['velocity'][t] = Aruco_Steering_Trajectories[run_name][mode]['velocity'][t]
				l = list(Aruco_Steering_Trajectories[run_name][mode]['other_car_inverse_distances'][t])
				l.reverse()
				Aruco_Steering_Trajectories[flip][mode]['other_car_inverse_distances'][t] = l
				l = list(Aruco_Steering_Trajectories[run_name][mode]['marker_inverse_distances'][t])
				l.reverse()
				Aruco_Steering_Trajectories[flip][mode]['marker_inverse_distances'][t] = l

	so(Aruco_Steering_Trajectories,opjD('Aruco_Steering_Trajectories.pkl'))

if True:
	print("Loading Aruco_Steering_Trajectories . . .")
	Aruco_Steering_Trajectories = lo(opjD('Aruco_Steering_Trajectories.pkl'))
	#Aruco_Steering_Trajectories = lo(opjD('Aruco_Steering_Trajectories_26May2017.pkl'))



if False:
	loss_dic = lo(opjD('loss_dic'))
	high_loss_dic = {}
	for k in loss_dic.keys():
		if loss_dic[k] >= 0.1:
			high_loss_dic[k] = loss_dic[k]


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
time.sleep(0)

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



loss_dic = {}
counter_dic = {}
counts = 0
high_loss_dic = {}
high_loss_keys = []
high_loss_key_ctr = 200000

def get_data_considering_high_low_steer_and_valid_trajectory_timestamp():
	global ctr_low
	global ctr_high
	global low_steer
	global high_steer
	global counts
	global high_loss_key_ctr
	global high_loss_keys

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


	run_name = get_data_with_hdf5.Segment_Data['run_codes'][run_code]
	if run_name not in Aruco_Steering_Trajectories.keys():
		#print('Run name '+run_name+' not in Aruco_Steering_Trajectories')
		return None
	if len(Aruco_Steering_Trajectories[run_name].keys()) != 4:
		return None

	#print 'here!'
	seg_num_str = str(seg_num)
	aruco_matches = []
	for i in [0]:#range(N_FRAMES):
		timestamp = get_data_with_hdf5.Segment_Data['runs'][run_name]['segments'][seg_num_str]['left_timestamp'][offset+i]
		behavioral_mode = np.random.choice(
			['Direct_Arena_Potential_Field',
 			'Furtive_Arena_Potential_Field',
 			'Follow_Arena_Potential_Field',
 			'Play_Arena_Potential_Field'])
		#print Aruco_Steering_Trajectories[run_name].keys()
		#print behavioral_mode
		#print run_name
		#print Aruco_Steering_Trajectories[run_name][behavioral_mode].keys()
		if timestamp in Aruco_Steering_Trajectories[run_name][behavioral_mode]['new_steer'].keys():
			aruco_matches.append(timestamp)
		if len(aruco_matches) < 1:
			return None
	#print aruco_matches
	if len(high_loss_dic) > 10000 and random.random() < 0.05:
		if high_loss_key_ctr >= 1000:
			high_loss_key_ctr = 0
			high_loss_keys = high_loss_dic.keys()
			np.random.shuffle(high_loss_keys)
		high_loss_key = high_loss_keys[high_loss_key_ctr]
		high_loss_key_ctr += 1
		

		#high_loss_key = random.choice(high_loss_dic.keys())
		run_name = high_loss_key[0]
		behavioral_mode = high_loss_key[1]
		timestamp = high_loss_key[2]
		run_code = high_loss_key[3]
		seg_num = high_loss_key[4]
		offset = high_loss_key[5]
	data = get_data_with_hdf5.get_data(run_code,seg_num,offset,N_STEPS,offset+0,N_FRAMES,ignore=ignore,require_one=require_one)
	if data != None:
		data['states'][0] = 1
		data['labels']['follow'] = False
		data['labels']['direct'] = False
		data['labels']['play'] = False
		data['labels']['furtive'] = False
		if behavioral_mode == 'Direct_Arena_Potential_Field':
			data['labels']['direct'] = True
		if behavioral_mode == 'Furtive_Arena_Potential_Field':
			data['labels']['furtive'] = True
		if behavioral_mode == 'Follow_Arena_Potential_Field':
			data['labels']['follow'] = True
		if behavioral_mode == 'Play_Arena_Potential_Field':
			data['labels']['play'] = True

		data['steer'] = array(data['steer'])*0.0 + Aruco_Steering_Trajectories[run_name][behavioral_mode]['new_steer'][timestamp]
 		data['id'] = (run_name,behavioral_mode,timestamp,run_code,seg_num,offset)
		if data['id'] not in counter_dic:
			counter_dic[data['id']] = 0
 		counter_dic[data['id']] += 1
 		counts += 1
		data['target_cars'] = Aruco_Steering_Trajectories[run_name][behavioral_mode]['other_car_inverse_distances'][timestamp]
		data['target_markers'] = Aruco_Steering_Trajectories[run_name][behavioral_mode]['marker_inverse_distances'][timestamp]
		data['target_velocity'] = Aruco_Steering_Trajectories[run_name][behavioral_mode]['velocity'][timestamp]
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
	histogram_plot_there = True
	clf()
	plt.hist(array(low_steer)[:,2],bins=range(0,100))
	plt.hist(array(high_steer)[:,2],bins=range(0,100))
	figure(1)

loss_threshold = 0.08
velocity_data = []
velocity_data_timer = Timer(60)
even_ctr = 0
while True:

	for b in range(Solver.batch_size):
		data = None
		while data == None:
			data = get_data_considering_high_low_steer_and_valid_trajectory_timestamp()
		Solver.put_data_into_model(data,Solver.solver,b)
	
	if Solver.solver.net.blobs['target_cars'].data[-1,:].max() == 0:
		continue
		if even_ctr == 0:
			continue
		elif even_ctr > 1:
			even_ctr = 0
	else:
		even_ctr += 1
	
	Solver.solver.step(1) # The training step. Everything below is for display.

	rate_ctr += 1
	if rate_timer.check():
		print(d2s('rate =',dp(rate_ctr/rate_timer_interval,2),'Hz'))
		rate_timer.reset()
		rate_ctr = 0
	the_loss = Solver.solver.net.blobs['steer_motor_target_data'].data[0,:] - Solver.solver.net.blobs['ip2'].data[0,:]
	the_loss = np.sqrt(the_loss * the_loss).mean()
	loss.append(the_loss)
	
	if the_loss >= loss_threshold:
		high_loss_dic[data['id']] = the_loss
	else:
		if data['id'] in high_loss_dic:
			del high_loss_dic[data['id']]
			#print(d2s('removed',data['id'],'from high_loss_dic'))
	loss_dic[data['id']] = the_loss
	
	if len(loss) >= 10000/Solver.batch_size:
		loss10000.append(array(loss[-10000:]).mean())
		loss = []
		if DISPLAY:
			figure('loss');clf()
			lm = min(len(loss10000),300)
			plot(loss10000[-lm:])
			if histogram_plot_there:
				plt.close('high low steer histograms')
				histogram_plot_there = False
		print(d2s('loss10000 =',loss10000[-1]))
	velocity_data.append([Solver.solver.net.blobs['target_velocity'].data[-1,0], Solver.solver.net.blobs['ip_velocity'].data[-1,0]])


	if print_timer.check():#Solver.solver.net.blobs['metadata'].data[0,3,0,0] > 0 and the_loss > 0.1:#loss_threshold:
		
		print(data['name'])
		print(Solver.solver.net.blobs['metadata'].data[-1,:,5,5])

		if Solver.solver.net.blobs['metadata'].data[0,2,0,0] > 0:
			print 'follow'
		if Solver.solver.net.blobs['metadata'].data[0,3,0,0] > 0:
			print 'direct'
		if Solver.solver.net.blobs['metadata'].data[0,4,0,0] > 0:
			print 'play'
		if Solver.solver.net.blobs['metadata'].data[0,5,0,0] > 0:
			print 'furtive'
		print(d2s('len(counter_dic),counts',(len(counter_dic),counts)))
		cprint(array_to_int_list(Solver.solver.net.blobs['steer_motor_target_data'].data[-1,:][:]),'green','on_red')
		cprint(array_to_int_list(Solver.solver.net.blobs['ip2'].data[-1,:][:]),'red','on_green')
		velocity_data.append([Solver.solver.net.blobs['target_velocity'].data[-1,0], Solver.solver.net.blobs['ip_velocity'].data[-1,0]])
		if DISPLAY:
			for plot_data in [['steer_motor_target_data','ip2'],
				['target_markers','ip_markers'],['target_cars','ip_cars'],]:
				figure(plot_data[0]);clf()
				t = Solver.solver.net.blobs[plot_data[0]].data[-1,:]
				o = Solver.solver.net.blobs[plot_data[1]].data[-1,:]
				ylim(-0.05,2.05);xlim(-0.5,len(t)-0.5)
				plot([-1,60],[0.49,0.49],'k');plot(o,'og'); plot(t,'or'); plt.title(data['name'])

			mi_or_cv2_animate(data['left'],delay=33);pause(0.001)

		print_timer.reset()
	
	if velocity_data_timer.check():
		velocity_data_timer.reset()
		figure('velocity data')
		clf()
		xylim(0,2,0,2)
		v = array(velocity_data)
		pts_plot(array(v))
		plt.title(d2s('r =',dp(np.corrcoef(v[:,0],v[:,1])[0,1],3)))

		if len(velocity_data) > 5000:
			velocity_data = velocity_data[-2500:]
	



