from kzpy3.utils import *
pythonpaths(['kzpy3','kzpy3/teg9'])
from vis import *
import data.utils.animate as animate
import arena.planner.Markers as Markers
import arena.planner.Potential_Fields as Potential_Fields
import arena.planner.Cars as Cars

bair_car_data_location = '/Volumes/SSD_2TB/bair_car_data_new_28April2017'


	

def get_sample_points(pts,angles,pfield,n=3):

    sample_points = []
    potential_values = []

    heading = normalized_vector_from_pts(pts[-n:,:])
    heading *= 0.5 # 50 cm, about the length of the car
    if pts[-n,0] > pts[-1,0]:
        heading *= -1
    #if pts[-3,1] > pts[-1,1]:
    #    heading *= -1

    for a in angles:

        sample_points.append( rotatePoint([0,0],heading,a) )
    #figure(3)
    #pts_plot(pts)
    for k in range(len(sample_points)):
        f = sample_points[k]
        #plot([pts[-1,0],pts[-1,0]+f[0]],[pts[-1,1],pts[-1,1]+f[1]])
    #figure(1)
    for sp in sample_points:
        pix = meters_to_pixels(sp[0]+pts[-1,0],sp[1]+pts[-1,1])
        #plot(pix[0],pix[1],'kx')
        potential_values.append(pfield[pix[0],pix[1]])

    return sample_points,potential_values


def interpret_potential_values(potential_values):
    min_potential_index = potential_values.index(min(potential_values))
    max_potential_index = potential_values.index(max(potential_values))
    middle_index = int(len(potential_values)/2)

    d = 99.0/(1.0*len(potential_values)-1)
    steer_angles = np.floor(99-arange(0,100,d))
    dp = potential_values[max_potential_index] - potential_values[min_potential_index]
    
    p = min(1,dp/max( (0.6-max(0,potential_values[max_potential_index]-0.8)) ,0.2) )
    steer = 99-int((p*steer_angles[min_potential_index]+(1-p)*49.0))
    return steer




def meters_to_pixels(x,y):
    return (int(-Mult*x)+Origin),(int(Mult*y)+Origin)






if __name__ == "__main__":
	DISPLAY_LEFT = True

		
	angles = range(-45,46,10)

	if 'N' not in locals():
		print("Loading trajectory data . . .")
		N = lo(opjD('N_pruned.pkl'))
	markers = Markers.Markers(Markers.markers_clockwise,4*107/100.)
	Origin = int(2*1000/300.*300 / 5)
	Mult = 1000/300.*50 / 5
	a = Potential_Fields.Play_Arena_Potential_Field(Origin,Mult,markers)
	a['Image']['img'] = z2o(a['Image']['img'])
	cars = {}
	for car_name in ['Mr_Black','Mr_Silver','Mr_Yellow','Mr_Orange','Mr_Blue']:
		cars[car_name] =  Cars.Car(N,car_name,Origin,Mult,markers)

	run_name = 'direct_rewrite_test_28Apr17_17h23m15s_Mr_Black'
	T0 = cars['Mr_Black']['runs'][run_name]['trajectory']['ts'][0]
	Tn = cars['Mr_Black']['runs'][run_name]['trajectory']['ts'][-1]
	loct = cars['Mr_Black']['runs'][run_name]['list_of_other_car_trajectories']
	cars['Mr_Black']['load_image_and_meta_data'](run_name,bair_car_data_location)
	timer = Timer(0)
	for car_name in cars:
		cars[car_name]['rewind']()
	figure(1,figsize=(12,12));clf();ds = 5;xylim(-ds,ds,-ds,ds)





	for car_name in cars:
		cars[car_name]['rewind']()
	for t in arange(T0+200,Tn,1/5.):
		#print(t)
		#if timer.time() > 1500:
		#	break
		p = cars['Mr_Black']['report_camera_positions'](run_name,t)
		other_cars_add_list = []

		if p != False:
			pass
			#pix = a['Image']['floats_to_pixels'](p[0])
			#a['Image']['img'][pix[0]-1:pix[0]+1,pix[1]-1:pix[1]+1] = 0

			#pt_plot(p[0],'r')
			#pt_plot(p[1],'r')
			#other_cars_add_list.append(p[0]) # TEMP
		
		for l in loct:
			other_car_name = l[0]
			other_car_run_name = l[1]
			p = cars[other_car_name]['report_camera_positions'](other_car_run_name,t)
			if p != False:
				#pix = a['Image']['floats_to_pixels'](p[0])
				#a['Image']['img'][pix[0]-1:pix[0]+1,pix[1]-1:pix[1]+1] = 5
				other_cars_add_list.append(p[0])
				#a['other_cars']([p[0]])
				pass
		a['other_cars'](other_cars_add_list)
		#mi(a['Image']['img']);
		img = a['Image']['img']
		width = shape(img)[0]
		origin = Origin
		mi(img[width/2-origin/2:width/2+origin/2,width/2-origin/2:width/2+origin/2],1)
		#other_cars_add_list = array(other_cars_add_list)
		#xy = other_cars_add_list*0
		#xy[:,0] = other_cars_add_list[:,1]
		#xy[:,1] = other_cars_add_list[:,0]
		#pts_plot(a['Image']['floats_to_pixels'](xy))

		pause(0.000001)
		if len(cars['Mr_Black']['pts']) > 3:
			sample_points,potential_values = get_sample_points(array(cars['Mr_Black']['pts']),angles,a['Image']['img'],3)
			steer = interpret_potential_values(potential_values)
			img = cars['Mr_Black']['get_left_image'](run_name).copy()
			print steer
			#mi(img,6,img_title=potential_values)
			#animate.prepare_and_show_or_return_frame(img,steer,0,6,66,1.0,cv2.COLOR_RGB2BGR)
			#apply_rect_to_img(img,steer,0,99,bar_color,bar_color,0.9,0.1,center=True,reverse=True,horizontal=True)
			animate.prepare_and_show_or_return_frame(img=img,steer=steer,motor=None,state=6,delay=66,scale=1,color_mode=cv2.COLOR_RGB2BGR)
			pause(0.06)
	#print timer.time()








