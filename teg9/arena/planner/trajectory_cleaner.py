from kzpy3.utils import *
pythonpaths(['kzpy3','kzpy3/teg9'])
from vis2 import *
from arena.planner.Constants import C
from data.utils.general import car_name_from_run_name






bair_car_data_location = C['bair_car_data_location']

if 'N' not in locals():
	print("Loading trajectory data . . .")
	N = lo(C['trajectory_data_location'])

# run_name = 'direct_rewrite_test_29Apr17_00h14m59s_Mr_Yellow' # wierd
# run_name = 'direct_rewrite_test_29Apr17_00h14m59s_Mr_Yellow' # wierd
#run_name = 'direct_rewrite_test_28Apr17_17h23m15s_Mr_Black' # 
run_name = 'direct_rewrite_test_28Apr17_18h09m52s_Mr_Black' # state flipping

car_name = car_name_from_run_name(run_name)

bag_folders_dst_rgb1to4_path = opj(bair_car_data_location,'rgb_1to4')
bag_folders_dst_meta_path = opj(bair_car_data_location,'meta')

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


figure(5);clf()

ref=XX('traj = N/`car_name/`run_name/self_trajectory'		) ;exec(ref)

ref=XX('left = traj/left'								) ;exec(ref)
ref=XX('right = traj/right'								) ;exec(ref)
ref=XX('N_ts = traj/ts'								) ;exec(ref)

#plot(traj['camera_separation'])
#plot(meo(left['t_vel'],10),'r-')
#plot(ts-ts[0],meo(right['t_vel'],10),'g-')
#plot(ts,meo((array(run_meta['motor'])-49)/6.0,10),'k')
#plot(ts,meo(run_meta['encoder'],30),'r-')
#plot(ts,array(run_meta['state'])/10.0,'c')
plot(ts-ts[0],(array(data['motor'])-49)/6.0,'k')
plot(ts-ts[0],(array(data['steer'])-49)/20.0,'b')
plot(ts-ts[0],data['encoder'],'r')
#plot(ts-ts[0],array(data['state'])/10.0,'y')
#plot(ts-ts[0],array(data['gyro'])/100.0,'b')
plot(ts-ts[0],array(data['gyro_heading'])/1000.0,'b')
##plot(ts-ts[0],array(data['acc'])/10.0,'r')

plot(N_ts-ts[0],left['t_vel'],'g')
#plot(N_ts-ts[0],left['x'],'c');plot(N_ts-ts[0],left['y'],'c')
#plot(N_ts-ts[0],right['x'],'c');plot(N_ts-ts[0],right['y'],'c')
#xlim(0,4000)
#ylim(-5,5)
#xylim(-5,5,-5,5)
pause(0.001)




def vec(heading,encoder):
	velocity = encoder/2.3
	"""
	if heading > 360:
		while heading > 360:
			heading -= 360
	elif heading < -360:
		while heading < -360:
			heading += 360
	"""
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





"""
>> N['Mr_Black']['direct_caffe_Fern_aruco_15Apr17_12h38m22s_Mr_Black']['other_trajectories']
N
0) Mr_Black:
	0) direct_caffe_Fern_aruco_15Apr17_12h38m22s_Mr_Black:
		0) other_trajectories:
			[direct_caffe_Fern_aruco_15Apr17_19h39m02s_Mr_Yellow direct_caffe_Fern_aruco_15Apr17_12h38m01s_Mr_Silver direct_caffe_Fern_aruco_15Apr17_12h38m05s_Mr_Blue direct_caffe_Fern_aruco_15Apr17_12h38m03s_Mr_Orange] (len=4)
		1) self_trajectory:
			0) camera_separation:
				[0.24 0.24 ... 863820.74 864383.35] (len=8435)
			1) left:
				0) t_vel:
					[0.0 0.07 ... 13633.12 14917.3] (len=8435)
				1) timestamp_gap:
					[0.04 0.04 ... 1.0 1.0] (len=8435)
				2) x:
					[-1.85 -1.84 ... 635883.74 636296.5] (len=8435)
				3) x_pix:
					[392.0 392.0 ... -31793887.0 -31814525.0] (len=8435)
				4) y:
					[2.14 2.14 ... 427337.66 427614.93] (len=8435)
				5) y_pix:
					[406.0 407.0 ... 21367182.0 21381046.0] (len=8435)

			2) right:
				0) t_vel:
					[0.0 0.04 ... 3348.89 3664.36] (len=8435)
				1) timestamp_gap:
					[0.04 0.04 ... 1.0 1.0] (len=8435)
				2) x:
					[-2.08 -2.08 ... -160618.08 -160724.36] (len=8435)
				3) x_pix:
					[403.0 403.0 ... 8031204.0 8036518.0] (len=8435)
				4) y:
					[2.08 2.08 ... 93015.77 93075.97] (len=8435)
				5) y_pix:
					[403.0 403.0 ... 4651088.0 4654098.0] (len=8435)

			3) run_name:
				 direct_caffe_Fern_aruco_15Apr17_12h38m22s_Mr_Black
			4) ts:
				[1492285137.42 1492285137.45 ... 1492285446.77 1492285446.81] (len=8435)


	..38)

..4)



"""