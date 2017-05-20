from kzpy3.vis import *
from kzpy3.teg9.data.markers_clockwise import markers_clockwise
import kzpy3.teg9.data.utils.get_new_A as get_new_A
#import operator
DISPLAY_LEFT = False
if DISPLAY_LEFT:
	import kzpy3.teg9.data.utils.multi_preprocess_pkl_files_1 as multi_preprocess_pkl_files_1





def car_name_from_run_name(rn):
	a = rn.split('Mr_')
	car_name = 'Mr_'+a[-1]
	car_name = car_name.replace('Mr_Yellow_A','Mr_Yellow')
	car_name = car_name.replace('Mr_Yellow_B','Mr_Yellow')
	return car_name



Origin = 300
Mult = 50
E = 10

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
		if traj[side]['t_vel'][i] > 2: # 1.788: # Above 4 mph
			c = (0,30,0)
		elif traj['camera_separation'][i] > 0.25: # almost larger than length of car
			c = (0,20,0)
		elif traj[side]['timestamp_gap'][i] > 0.1: # missed data points
			c = (0,10,0,0)		
		cv2.circle(out_img,(traj[side]['x_pix'][i],traj[side]['y_pix'][i]),1,c,-1)


#/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017/meta/direct_rewrite_test_30Apr17_12h29m10s_Mr_Black
if DISPLAY_LEFT:
	bag_folders_dst_rgb1to4_path = '/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017/rgb_1to4'
	bag_folders_dst_meta_path = '/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017/meta'# opjD('bair_car_data_new/meta')# '/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017/meta'

N = lo(opjD('N.pkl'))




while True:
	try:
		traj_lst = []
		CAR_NAME = random.choice(N.keys())
		RUN_NAME = random.choice(N[CAR_NAME].keys())
		if len(N[CAR_NAME][RUN_NAME]['other_trajectories']) < 2:
			continue
		for ot in [N[CAR_NAME][RUN_NAME]['self_trajectory']]+N[CAR_NAME][RUN_NAME]['other_trajectories']:
			run_name = ot['run_name']
			car_name = car_name_from_run_name(run_name)
			traj_lst.append( N[car_name][run_name]['self_trajectory'] )
			print(car_name,run_name)

		for i in range(4):
			traj_lst[i]['data'] = get_new_A.get_new_A()
			if DISPLAY_LEFT:
				multi_preprocess_pkl_files_1.multi_preprocess_pkl_files(
					traj_lst[i]['data'],
						opj(bag_folders_dst_meta_path,traj_lst[i]['run_name']),
						opj(bag_folders_dst_rgb1to4_path,traj_lst[i]['run_name']))

		T0 = traj_lst[0]['ts'][0]
		t = T0
		Tn = traj_lst[0]['ts'][-1]


		DT = 1/30.
		dt = DT
		timer = Timer(10)
		PAUSE = False

		out_img *= 0
		draw_markers(out_img)
		cv2.putText(out_img,RUN_NAME,(50,2*Origin-50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255));

		while t < traj_lst[0]['ts'][-1]:

			if not PAUSE:
				if timer.check():
					out_img *= 0
					draw_markers(out_img)
					cv2.putText(out_img,RUN_NAME,(50,2*Origin-50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255));
					timer.reset()

			ctr = 0
			for traj in traj_lst:
				car_name = car_name_from_run_name(traj['run_name'])
				if t>traj['ts'][0] and t<traj['ts'][-1]:
					near_t = -1
					for i in range(1,len(traj['ts'])):
						if traj['ts'][i-1]<t and traj['ts'][i]>t:
							near_t = traj['ts'][i]
							near_i = i
							break
					if near_t > 0:
						for side in ['left','right']:
							plot_trajectory_point(traj,side,near_i,near_t,out_img,colors[car_name])
						if ctr < 4 and DISPLAY_LEFT:
							quadrant = ctr
							index = traj['data']['t_to_indx'][near_t]
							img = traj['data']['left'][index]
							if quadrant == 0:
								out_img[:shape(img)[0]+E,:E+shape(img)[1],:] = colors[car_name]
								out_img[:shape(img)[0],:shape(img)[1]] = img
							elif quadrant == 1:
								out_img[-E-shape(img)[0]:,:E+shape(img)[1],:] = colors[car_name]
								out_img[-shape(img)[0]:,:shape(img)[1]] = img
							elif quadrant == 2:
								out_img[:shape(img)[0]+E,-E-shape(img)[1]:,:] = colors[car_name]
								out_img[:shape(img)[0]:,-shape(img)[1]:] = img
							elif quadrant == 3:
								out_img[-E-shape(img)[0]:,-E-shape(img)[1]:,:] = colors[car_name]
								out_img[-shape(img)[0]:,-shape(img)[1]:] = img

				ctr += 1

			k = mci(out_img,delay=33)


			if not PAUSE:
				dt = DT
			if k == ord('q'):
				print('q')
				break
			if k == ord('d'):
				print('done')
				exit()
			elif k == ord('k'):
				dt = -2
			elif k == ord('l'):
				dt = 2
			elif k == ord(' '):
				if PAUSE:
					PAUSE = False
					print("<<end pause>>")
				else:
					PAUSE = True
					dt = 0
					print("<<pause>>")
			elif k == ord('d'):
				print('done')
				exit()

			if abs(dt) > DT:
				timer.trigger()

			t += dt

			if t < T0:
				t = T0
			elif t >= Tn:
				print('At end')
				t = Tn-1




	except Exception as e:
		print("********** Exception ***********************")
		print(time_str('Pretty'))
		print(e.message, e.args)







