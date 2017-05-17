from kzpy3.vis import *
from kzpy3.teg9.data.markers_clockwise import markers_clockwise
import operator
import kzpy3.teg9.data.utils.multi_preprocess_pkl_files_1 as multi_preprocess_pkl_files_1
import kzpy3.teg9.data.utils.get_trajectory_points as get_trajectory_points

Origin = 300
Mult = 50

out_img = zeros((Origin*2,Origin*2,3),np.uint8)

marker_ids_all = []
marker_angles_dic = {}
marker_angles = 2*np.pi*np.arange(len(markers_clockwise))/(1.0*len(markers_clockwise))
marker_xys = []
for i in range(len(markers_clockwise)):
	a = marker_angles[i]
	marker_angles_dic[markers_clockwise[i]] = a
	x = 4*107/100.*np.sin(a)
	y = 4*107/100.*np.cos(a)
	marker_xys.append([x,y])

markers_xy_dic = {}
assert(len(markers_clockwise) == len(marker_xys))

def meters_to_pixels(x,y):
	return (int(-Mult*x)+Origin),(int(Mult*y)+Origin)

def draw_markers(out_img):
	for j in range(len(markers_clockwise)):
		m = markers_clockwise[j]
		xy = marker_xys[j]
		markers_xy_dic[m] = xy
		c = (255,0,0)
		xp,yp = meters_to_pixels(xy[0],xy[1])
		cv2.circle(out_img,(xp,yp),4,c,-1)



colors = {'Mr_Yellow':(255,255,0), 'Mr_Silver':(255,255,255), 'Mr_Blue':(0,0,255), 'Mr_Orange':(255,0,0), 'Mr_Black':(100,100,100)}



def plot_trajectory_point(traj,side,i,t,out_img,c):
	assert(traj['ts'][i] <= t)
	if traj['ts'][i] == t:
		if traj[side]['t_vel'][i] > 1.788: # Above 4 mph
			c = (0,30,0)
		elif traj['camera_separation'][i] > 0.25: # almost larger than length of car
			c = (0,20,0)
		elif traj[side]['timestamp_gap'][i] > 0.1: # missed data points
			c = (0,10,0,0)		
		cv2.circle(out_img,(traj[side]['x_pix'][i],traj[side]['y_pix'][i]),1,c,-1)



N = lo(opjh('caffe_models','N.pkl'))


CAR_NAME = 'Mr_Blue'
RUN_NAME = 'direct_rewrite_test_28Apr17_17h23m10s_Mr_Blue'


traj1 = N[CAR_NAME][RUN_NAME]['self_trajectory']
traj2 = N[CAR_NAME][RUN_NAME]['other_trajectories']

out_img *= 0
draw_markers(out_img)

for i in range(len(ts)):
	for traj in [traj1]+traj2:
		t = traj['ts'][i]
		for side in ['left','right']:
			car_name = get_trajectory_points.car_name_from_run_name(traj['run_name'])
			plot_trajectory_point(traj,side,i,t,out_img,colors[car_name])
		k = mci(out_img,delay=1)
		if k == ord('q'):
			break



