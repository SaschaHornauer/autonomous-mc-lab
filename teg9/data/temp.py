from kzpy3.vis import *
from kzpy3.data_analysis.markers_clockwise import markers_clockwise



out_img = zeros((1000,1000,3),np.uint8)



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
for i in range(len(markers_clockwise)):
	m = markers_clockwise[i]
	xy = marker_xys[i]
	markers_xy_dic[m] = xy
	cv2.circle(out_img,(int(100*xy[0])+500,int(100*xy[1])+500),4,(255,0,0),-1)





def plot_it2(angle1,distance1,angle2,distance2,xy):

	xd = distance1 * np.sin(angle2)
	yd = distance1 * np.cos(angle2)

	plot([xy[0],xd+xy[0]],[xy[1],yd+xy[1]])

	pause(0.001)

#Mr_Black_marker_data_pkl = lo('/home/karlzipser/Desktop/bair_car_data_new/meta/direct_rewrite_test_28Apr17_17h23m15s_Mr_Black/marker_data.pkl')
#Mr_Blue_marker_data_pkl = lo('/home/karlzipser/Desktop/bair_car_data_new/meta/direct_rewrite_test_28Apr17_17h23m10s_Mr_Blue/marker_data.pkl')

#Mr_Silver_marker_data_pkl = lo('/home/karlzipser/Desktop/bair_car_data_new/meta/direct_rewrite_test_28Apr17_18h12m54s_Mr_Silver/marker_data.pkl' )

#Mr_Orange_marker_data_pkl = lo('/home/karlzipser/Desktop/bair_car_data_new/meta/direct_rewrite_test_25Apr17_14h32m22s_Mr_Orange/marker_data.pkl' )
#A = lo('/home/karlzipser/Desktop/bair_car_data_new/meta/direct_rewrite_test_25Apr17_13h09m04s_Mr_Black/marker_data.pkl')
#A = lo('/home/karlzipser/Desktop/bair_car_data_new/meta/direct_rewrite_test_25Apr17_13h09m04s_Mr_Black/marker_data.pkl')
#A = lo('/home/karlzipser/Desktop/bair_car_data_new/meta/direct_rewrite_test_25Apr17_21h31m25s_Mr_Yellow/marker_data.pkl')
#marker_data_pkl = Marker_Raw_Data['Mr_Yellow']['direct_rewrite_test_25Apr17_21h31m25s_Mr_Yellow']
#Mr_Yellow_marker_data_pkl = lo('/home/karlzipser/Desktop/bair_car_data_new/meta/direct_rewrite_test_25Apr17_21h31m25s_Mr_Yellow/marker_data.pkl')



def init_car_path(marker_data_pkl,side,name):
	A = {}
	A['marker_data_pkl'] = marker_data_pkl[side]
	A['x_avgs'] = []
	A['y_avgs'] = []
	A['pts'] = []
	A['ts'] = sorted(A['marker_data_pkl'].keys())
	A['timestamp_index'] = 0
	A['name'] = name
	return A





def get_position_and_heading(angles_to_center,angles_surfaces,distances_marker):
	marker_ids = angles_to_center.keys()
	x_avg = 0.0
	y_avg = 0.0
	d_sum = 0.0
	xs = []
	ys = []
	ds = []
	headings = []

	for m in marker_ids:
		if m in [190]: # This one gives false positives on ground.
			continue
		
		if m in markers_xy_dic:
			xy = markers_xy_dic[m]
			angle1 = angles_to_center[m]
			distance1 = distances_marker[m]
			distance2 = 4*107/100.
			angle2 = (np.pi+marker_angles_dic[m]) - (np.pi/2.0-angles_surfaces[m])
			xd = distance1 * np.sin(angle2)
			yd = distance1 * np.cos(angle2)
			#print (dp(np.degrees(marker_angles_dic[m]+np.pi/2.0-angles_surfaces[m]+angles_to_center[m]),2))#,dp(np.degrees(marker_angles_dic[m]),2),dp(np.degrees(angles_surfaces[m]),2),dp(np.degrees(angles_to_center[m],2)))

			if distance1 < 2*distance2 and distance1 > 0.05:
				xs.append(xd+xy[0])
				ys.append(yd+xy[1])
				ds.append(distance1)
				headings.append(marker_angles_dic[m]+angles_surfaces[m]-np.pi/2.0+angles_to_center[m])

	d = 0
	for i in range(len(xs)):
		d += 1/ds[i]
		x_avg += d*xs[i]
		y_avg += d*ys[i]
		heading_avg = d*headings[i]
		d_sum += d

	median_d = np.median(array(ds))
	if d_sum == 0:
		return None,None,None,None
	x_avg /= d_sum
	y_avg /= d_sum
	heading_avg /= d_sum
	median_d = max(median_d,1)
	median_d = median_d**2+3



	return marker_ids,x_avg,y_avg,median_d,heading_avg












def show_timepoint(A,timestamp,out_img,dot_color,max_dt=60/1000.,start_index=0,hour_correction=0):
	try:
		A['timestamp_index'] = start_index
		while A['ts'][A['timestamp_index']] < timestamp:	
			A['timestamp_index'] += 1
		if A['ts'][A['timestamp_index']]-timestamp > max_dt:
			print A['ts'][A['timestamp_index']]-timestamp
			return

		i = A['timestamp_index']
		ts = A['ts']

		angles_to_center = A['marker_data_pkl'][ts[i]]['angles_to_center']
		angles_surfaces = A['marker_data_pkl'][ts[i]]['angles_surfaces']
		distances_marker = A['marker_data_pkl'][ts[i]]['distances_marker']

		marker_ids,x_avg,y_avg,median_d,heading = get_position_and_heading(angles_to_center,angles_surfaces,distances_marker)
		if marker_ids == None:
			return
		A['x_avgs'].append(x_avg)
		A['y_avgs'].append(y_avg)
		if len(A['x_avgs'])>10:
			x = array(A['x_avgs'][-int(5*median_d):]).mean()
			y = array(A['y_avgs'][-int(5*median_d):]).mean()
			A['pts'].append([x,y])
			if len(A['pts']) > 100:
				A['pts'] = A['pts'][-100:]
			if len(A['pts'])>11:
				for qq in range(10,0,-1):
					x,y = A['pts'][-qq][0],A['pts'][-qq][1]
					new_dot_color = array(dot_color)/(np.sqrt(qq))
					x2,y2 = x+0.4*np.sin(heading),y+0.4*np.cos(heading)
					cv2.line(out_img,(int(-100*x)+500,int(100*y)+500),(int(-100*x2)+500,int(100*y2)+500),(255,255,255))
					cv2.circle(out_img,(int(-100*x)+500,int(100*y)+500),4,new_dot_color,-1)
			for j in range(len(markers_clockwise)):
				m = markers_clockwise[j]
				xy = marker_xys[j]
				markers_xy_dic[m] = xy
				c = (255,0,0)
				if m in marker_ids:
					c = (0,255,0)
					cv2.circle(out_img,(int(-100*xy[0])+500,int(100*xy[1])+500),4,c,-1)
		k = mci(out_img,delay=1,title='out_img')
		if k == ord('q'):
			return;	
		if len(A['x_avgs']) > 100:
			A['x_avgs'] = A['x_avgs'][-100:]
			A['y_avgs'] = A['y_avgs'][-100:]
	except:
		time.sleep(0.01)

"""
Yl = init_car_path(Mr_Yellow_marker_data_pkl,'left','Mr_Yellow')
Yr = init_car_path(Mr_Yellow_marker_data_pkl,'right','Mr_Yellow')

Ol = init_car_path(Mr_Orange_marker_data_pkl,'left','Mr_Orange')
Or = init_car_path(Mr_Orange_marker_data_pkl,'right','Mr_Orange')

#Sl = init_car_path(Mr_Silver_marker_data_pkl,'right','Mr_Orange')
#Sr = init_car_path(Mr_Silver_marker_data_pkl,'right','Mr_Orange')

Bkl = init_car_path(Mr_Black_marker_data_pkl,'left','Mr_Black')
Bkr = init_car_path(Mr_Black_marker_data_pkl,'right','Mr_Black')
"""
Bul = init_car_path(Mr_Blue_marker_data_pkl,'left','Mr_Blue')
Bur = init_car_path(Mr_Blue_marker_data_pkl,'right','Mr_Blue')

out_img *= 0
iprev = 0
tprev = 0
for i in range(len(Bul['ts'])):
	t =  i/30.
	#print t
	t += Bul['ts'][0]
	for j in range(len(markers_clockwise)):
		m = markers_clockwise[j]
		xy = marker_xys[j]
		markers_xy_dic[m] = xy
		c = (255,0,0)
		cv2.circle(out_img,(int(-100*xy[0])+500,int(100*xy[1])+500),4,c,-1)
	show_timepoint(Bul,t,out_img,(0,0,255),100000.,0,-7)
	#show_timepoint(Bur,t,out_img,(0,100,255),100000.,0,0)
	#show_timepoint(Bkl,t,out_img,(255,100,0),100000.,0,-7)
	#show_timepoint(Bkr,t,out_img,(255,0,0),100000.,0,0)
	tprev = t
	iprev = i
	if t-tprev > 1:
		print tprev
	