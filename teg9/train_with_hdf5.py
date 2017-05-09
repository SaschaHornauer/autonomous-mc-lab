import kzpy3.teg9.get_data_with_hdf5 as get_data_with_hdf5








bair_car_data_path = '/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017'#opjD('bair_car_data_new')
hdf5_runs_path = opj(bair_car_data_path,'hdf5/runs')
hdf5_segment_metadata_path = opj(bair_car_data_path,'hdf5/segment_metadata')








get_data_with_hdf5.load_Segment_Data(hdf5_segment_metadata_path,hdf5_runs_path)









print('\nloading low_steer... (takes awhile)')
low_steer = load_obj(opj(bair_car_data_path,'hdf5/segment_metadata/low_steer'))
print('\nloading high_steer... (takes awhile)')
high_steer = load_obj(opj(bair_car_data_path,'hdf5/segment_metadata/high_steer'))

len_high_steer = len(high_steer)
len_low_steer = len(low_steer)

figure('high low steer histograms')
clf()
plt.hist(array(low_steer)[:,2],bins=range(0,100))
plt.hist(array(high_steer)[:,2],bins=range(0,100))
figure(1)
ctr_low = -1 # These counter keep track of position in segment lists, and when to reshuffle.
ctr_high = -1








if True:

	N_FRAMES = 2 # how many timesteps with images.
	N_STEPS = 10 # how many timestamps with non-image data

	if 'solver_state_1_5_6_7_plus_extra_Smyth_racing' in solver_name:
		ignore = [reject_run,left,out1_in2] # runs with these labels are ignored
		require_one = [Smyth,racing] # at least one of this type of run lable is required
		use_states = [1,5,6,7]

	print_timer = Timer(5)
	loss10000 = []
	loss = []
	rate_timer_interval = 10.
	rate_timer = Timer(rate_timer_interval)
	rate_ctr = 0
	figure('steer',figsize=(3,2))
	figure('loss',figsize=(3,2))
	while True:

		
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
			
		if random.random() > 0.5: # with some probability choose a low_steer element
			choice = low_steer[ctr_low]
			ctr_low += 1
		else:
			choice = high_steer[ctr_high]
			ctr_high += 1
		run_code = choice[3]
		seg_num = choice[0]
		offset = choice[1]
		data = get_data_with_hdf5.get_data(run_code,seg_num,offset,N_STEPS,offset+0,N_FRAMES,ignore=ignore,require_one=require_one,use_states=use_states)
		if data == None:
			continue



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


		
		solver.step(1) # The training step. Everything below is for display.
		rate_ctr += 1
		if rate_timer.check():
			print(d2s('rate =',dp(rate_ctr/rate_timer_interval,2),'Hz'))
			rate_timer.reset()
			rate_ctr = 0
		a = solver.net.blobs['steer_motor_target_data'].data[0,:] - solver.net.blobs['ip2'].data[0,:]
		loss.append(np.sqrt(a * a).mean())
		if len(loss) >= 10000:
			loss10000.append(array(loss[-10000:]).mean())
			loss = []
			figure('loss');clf()
			lm = min(len(loss10000),100)
			plot(loss10000[-lm:])
			print(d2s('loss10000 =',loss10000[-1]))
		if print_timer.check():
			print(solver.net.blobs['metadata'].data[0,:,5,5])
			cprint(array_to_int_list(solver.net.blobs['steer_motor_target_data'].data[0,:][:]),'green','on_red')
			cprint(array_to_int_list(solver.net.blobs['ip2'].data[0,:][:]),'red','on_green')
			figure('steer')
			clf()
			xlen = len(solver.net.blobs['ip2'].data[0,:][:])/2-1
			ylim(-5,105);xlim(0,xlen)
			t = solver.net.blobs['steer_motor_target_data'].data[0,:]*100.
			o = solver.net.blobs['ip2'].data[0,:]*100.
			plot(zeros(xlen+1)+49,'k');plot(o,'g'); plot(t,'r'); plt.title(data['name']);pause(0.001)
			mi_or_cv2_animate(data['left'],delay=60)
			print_timer.reset()

