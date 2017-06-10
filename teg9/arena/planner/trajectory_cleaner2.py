from kzpy3.utils2 import *
pythonpaths(['kzpy3','kzpy3/teg9'])
from vis2 import *
from arena.planner.Constants import C
from data.utils.general import car_name_from_run_name

bair_car_data_location = C['bair_car_data_location']

if 'N' not in locals():
	print("Loading trajectory data . . .")
	N = lo(C['trajectory_data_location'])
 
run_name = 'direct_rewrite_test_28Apr17_18h09m52s_Mr_Black'





def get_metadata(run_name,bair_car_data_location):
	L = lo(opj(bair_car_data_location,'meta',run_name,'left_image_bound_to_data.pkl'))
	ts,data_list = get_sorted_keys_and_data(L)
	ts = array(ts)
	data_types = data_list[0].keys()
	data = {}
	for d in data_types:
		data[d] = []
	for e in data_list:
		for d in data_types:
			data[d].append(e[d])
	for d in data_types:
		data[d] = array(data[d])
	return ts,data

def time_correct_traj(run_name,N):
	"""
	There is a time offset between the trajectory data and the older metadata timestamps.
	"""
	car_name = car_name_from_run_name(run_name)
	assert('TIME_CORRECTION_DONE' not in N[car_name][run_name])
	traj = N[car_name][run_name]['self_trajectory']
	traj['ts'] = traj['ts'][30:-30]
	for i in range(len(traj['ts'])):
		assert(traj['ts'][i]) == ts[i]
	traj['camera_separation'] = traj['camera_separation'][30:-30]
	for side in ['left','right']:
		for e in ['x','y','t_vel','timestamp_gap']:
			traj[side][e] = traj[side][e][30:-30]
	N[car_name][run_name]['TIME_CORRECTION_DONE'] = True
	return traj

def check_trajectory_point(traj,side,i,t):
	if traj[side]['t_vel'][i] > 3: # 1.788: # Above 4 mph
		return False
	if traj[side]['t_vel'][i]<0.1: #TEMP
		return False
	elif traj['camera_separation'][i] > 0.5: # almost larger than length of car
		return False
	elif traj[side]['timestamp_gap'][i] > 0.5: # missed data points
		return False
	elif length([traj[side]['x'][i],traj[side]['y'][i]]) > C['Marker_Radius']:
		return False
	return True

def get_traj_valid(traj,ts):
	traj_valid = []
	for i in range(len(ts)):
		t = ts[i]
		valid = True
		for side in ['left','right']:
			if not check_trajectory_point(traj,side,i,t):
				valid = False
				break
		if valid:
			v = 1
		else:
			v = 0
		traj_valid.append(v)
	return array(traj_valid)


def get_invalid_states(traj_valid):
	invalid_states = []
	invalid_state = [0,0]
	waiting_for_first_valid = 1
	waiting_for_valid = 2
	waiting_for_invalid = 3
	vstate = waiting_for_first_valid
	for i in rlen(traj_valid):
		if traj_valid[i] == 1:
			if vstate == waiting_for_first_valid:
				vstate = waiting_for_invalid
			elif vstate == waiting_for_invalid:
				pass
			elif vstate == waiting_for_valid:
				invalid_state[1] = i-1
				invalid_states.append(invalid_state)
				invalid_state = [0,0]
				vstate = waiting_for_invalid
			else:
				assert(False)
		elif traj_valid[i] == 0:
			if vstate == waiting_for_first_valid:
				pass
			elif vstate == waiting_for_invalid:
				invalid_state[0] = i
				print invalid_state
				vstate = waiting_for_valid
			elif vstate == waiting_for_valid:
				pass
			else:
				assert(False)
	return invalid_states

def interpolate_over_invalid(traj,invalid_states):
	for c in ['x','y']:
		for invalid_state in invalid_states:
			start = traj[c][invalid_state[0]-1]
			end = traj[c][invalid_state[1]+1]
			l = invalid_state[1]-invalid_state[0]+1
			for j in range(l):
				traj[c][invalid_state[0]+j] = start + j/(1.0*l)*(end-start)

def interpolate_over_still(traj,data):
	meoencoder = array(meo(data['encoder'],120))
	for d in ['forward','backward']:
		for c in ['x','y']:
			traj[d+'_'+c] = traj[c].copy()
	still = False
	for i in range(0,len(ts)):
		if meoencoder[i] > 0.01:
			still = False
		else:
			if still == False:
				still = True
				x = traj['forward_x'][i-1]
				y = traj['forward_y'][i-1]
			traj['forward_x'][i] = x
			traj['forward_y'][i] = y
	still = False
	for i in range(len(ts)-2,0,-1):
		if meoencoder[i] > 0.01:
			still = False
		else:
			if still == False:
				still = True
				x = traj['backward_x'][i+1]
				y = traj['backward_y'][i+1]
			traj['backward_x'][i] = x
			traj['backward_y'][i] = y
	traj['new_x'] = array(meo((traj['backward_x']+traj['forward_x'])/2.0,60))
	traj['new_y'] = array(meo((traj['backward_y']+traj['forward_y'])/2.0,60))




#def get_headings(traj):
traj['heading'] = []
traj['absolute_heading'] = []
traj['relative_heading'] = []
n = C['n_for_heading']
pts = array(zip(traj['new_x'],traj['new_y']))
for i in range(len(traj['new_x'])):
	if i <= n:
		traj['heading'].append(array([0,1]))
		traj['relative_heading'].append(0)
	else:
		#print('here')
		traj['heading'].append(normalized_vector_from_pts(pts[i-n+1:i]))
		if pts[i-n][0] > pts[i][0]:
			traj['heading'][i] *= -1.0
		#if i > n+1 and np.degrees(angle_between(traj['heading'][i],traj['heading'][i-1])) > 45:
		#		traj['heading'][i] = traj['heading'][i-1]
		traj['relative_heading'].append(angle_clockwise(traj['heading'][i],pts[i]))
		traj['absolute_heading'].append(angle_clockwise(traj['heading'][i],[0,1]))





ts,data = get_metadata(run_name,bair_car_data_location)

traj = time_correct_traj(run_name,N)

traj_valid = get_traj_valid(traj,ts)

for c in ['x','y']:
	traj[c] = (traj['left'][c]+traj['right'][c])/2.0 * traj_valid

invalid_states = get_invalid_states(traj_valid)

interpolate_over_invalid(traj,invalid_states)

interpolate_over_still(traj,data)






GRAPHICS = True
if GRAPHICS:
	figure(5);clf()
	plot(ts-ts[0],(array(data['motor'])-49)/6.0,'k')
	plot(ts-ts[0],(array(data['steer'])-49)/20.0,'b')
	plot(ts-ts[0],data['encoder'],'r')
	plot(ts-ts[0],array(data['gyro_heading'])/1000.0,'b')
	pause(0.001)
	plot(ts-ts[0],traj_valid,'.')

############## drive from heading test
#
def vec(heading,encoder):
	velocity = encoder/2.3 # rough guess
	a = [0,1]
	a = array(rotatePoint([0,0],a,heading))
	a *= velocity/30.0
	return array(a)
figure(99);clf()
plt_square();
xylim(-15,15,-15,15)
xy = array([0.0,0.0])
xys=[]
for i in range(len(ts)):
	#plot(xy[0],xy[1],'r.')
	heading = data['gyro_heading'][i][0]
	encoder = data['encoder'][i]
	v = vec(heading,encoder)
	xy += v
	xys.append(array(xy))
	print i#(heading,encoder,v)
	#pause(0.0001)
pts_plot(array(xys))
#
##############