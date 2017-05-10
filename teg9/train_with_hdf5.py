REPO = 'kzpy3'
TEG = 'teg9'

#exec('from '+REPO+'.utils import *')
#exec(d2n("import ",REPO,".",TEG,".get_data_with_hdf5 as get_data_with_hdf5"))

from kzpy3.utils import *
import kzpy3.teg9.get_data_with_hdf5 as get_data_with_hdf5

os.environ['GLOG_minloglevel'] = '2'

import caffe

gpu = 1
if gpu >= 0:
	caffe.set_device(gpu)
	caffe.set_mode_gpu()


CAF = 'caf8'
MODEL = 'zn_color'
#exec('from '+REPO+'.'+CAF+'.'+MODEL+'.solver import solver,model_path,put_data_into_model')
import kzpy3.caf8.zn_color.solver as Solver

model_path = Solver.model_path

weights_file_path =  most_recent_file_in_folder(opjD(fname(model_path)))
weights_file_path = None

if weights_file_path:
	print(d2s("Copying weights from",weights_file_path,"to",Solver.solver))
	Solver.solver.net.copy_from(weights_file_path)
else:
	print(d2s("No weights loaded to",Solver.solver))




bair_car_data_path = opjD('bair_car_data_new') # '/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017'#opjD('bair_car_data_new')
hdf5_runs_path = opj(bair_car_data_path,'hdf5/runs')
hdf5_segment_metadata_path = opj(bair_car_data_path,'hdf5/segment_metadata')

N_FRAMES = 10 # how many timesteps with images.
N_STEPS = 40 # how many timestamps with non-image data
ignore = ['reject_run','left','out1_in2'] # runs with these labels are ignored
require_one = ['direct'] # at least one of this type of run lable is required
use_states = [1,5,6,7]

print_timer = Timer(5)
loss10000 = []
loss = []
rate_timer_interval = 10.
rate_timer = Timer(rate_timer_interval)
rate_ctr = 0
figure('steer',figsize=(3,2))
figure('loss',figsize=(3,2))



get_data_with_hdf5.load_Segment_Data(hdf5_segment_metadata_path,hdf5_runs_path)



print('\nloading low_steer... (takes awhile)')
low_steer = load_obj(opj(bair_car_data_path,'hdf5/segment_metadata/low_steer'))
print('\nloading high_steer... (takes awhile)')
high_steer = load_obj(opj(bair_car_data_path,'hdf5/segment_metadata/high_steer'))
len_high_steer = len(high_steer)
len_low_steer = len(low_steer)
figure('high low steer histograms',figsize=(2,1))
clf()
plt.hist(array(low_steer)[:,2],bins=range(0,100))
plt.hist(array(high_steer)[:,2],bins=range(0,100))
figure(1)
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
		
	if random.random() < len_high_steer/(len_low_steer+len_high_steer+0.0): # with some probability choose a low_steer element
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



batch_size = 1

while True:

	data = get_data_considering_high_low_steer()
	if data == None:
		continue
	Solver.put_data_into_model(data,Solver.solver)

	Solver.solver.step(1)

	# The training step. Everything below is for display.
	rate_ctr += 1
	if rate_timer.check():
		print(d2s('rate =',dp(rate_ctr/rate_timer_interval,2),'Hz'))
		rate_timer.reset()
		rate_ctr = 0
	a = Solver.solver.net.blobs['steer_motor_target_data'].data[0,:] - Solver.solver.net.blobs['ip2'].data[0,:]
	loss.append(np.sqrt(a * a).mean())
	if len(loss) >= 10000:
		loss10000.append(array(loss[-10000:]).mean())
		loss = []
		figure('loss');clf()
		lm = min(len(loss10000),100)
		plot(loss10000[-lm:])
		print(d2s('loss10000 =',loss10000[-1]))
	if print_timer.check():
		print(Solver.solver.net.blobs['metadata'].data[0,:,5,5])
		cprint(array_to_int_list(Solver.solver.net.blobs['steer_motor_target_data'].data[0,:][:]),'green','on_red')
		cprint(array_to_int_list(Solver.solver.net.blobs['ip2'].data[0,:][:]),'red','on_green')
		figure('steer')
		clf()
		xlen = len(Solver.solver.net.blobs['ip2'].data[0,:][:])/2-1
		ylim(-5,105);xlim(0,xlen)
		t = Solver.solver.net.blobs['steer_motor_target_data'].data[0,:]*100.
		print Solver.solver.net.blobs['steer_motor_target_data'].data[0,:]
		o = Solver.solver.net.blobs['ip2'].data[0,:]*100.
		plot(zeros(xlen+1)+49,'k');plot(o,'g'); plot(t,'r'); plt.title(data['name']);pause(0.001)
		mi_or_cv2_animate(data['left'],delay=33)
		print_timer.reset()





