from kzpy3.utils import *
pythonpaths(['kzpy3','kzpy3/teg9'])
from vis2 import *
import data.utils.animate as animate
import arena.planner.Markers as Markers
import arena.planner.Potential_Fields as Potential_Fields
import arena.planner.Cars as Cars


def Run(run_name,cars,the_arena):
	D = {}
	D['Purpose'] = d2s(inspect.stack()[0][3],':','Run object.')	
	def _setup():

		if len(gg(opjD(bair_car_data_location,'meta',run_name,'*'))) < 5: #'caffe2_z2_color_direct_local_01Jan13_00h01m07s_Mr_Yellow' in run_name:
			print("len(gg(opjD(bair_car_data_location,'meta',run_name,'*'))) < 5")
			continue
		velocity = (cars[our_car]['runs'][run_name]['trajectory']['left']['t_vel']+cars[our_car]['runs'][run_name]['trajectory']['right']['t_vel'])/2.0
		#velocity = mean_exclude_outliers(velocity,15,1/3.0,2/3.0)
		

		

			print(d2n(our_car,'\n\t',run_name))
			zaccess(N[our_car][run_name],[0]);

			T0 = cars[our_car]['runs'][run_name]['trajectory']['ts'][0]
			Tn = cars[our_car]['runs'][run_name]['trajectory']['ts'][-1]
			list_of_other_car_trajectories = cars[our_car]['runs'][run_name]['list_of_other_car_trajectories']
			try:
				cars[our_car]['load_image_and_meta_data'](run_name,bair_car_data_location)
			except Exception as e:
				print("********** Exception *** cars[our_car]['load_image_and_meta_data'](run_name,bair_car_data_location) ********************")
				print(our_car,run_name)
				print(e.message, e.args)


				

				
					
					
	def _step(t):

					
					t_prev = t
					if timer.check():
						print(time_str('Pretty'))
						timer.reset()





						if no_cars_in_view:
							pass#continue

						the_arena['other_cars'](other_cars_in_view_xy_list,mode,xy_our)
						img = the_arena['Image']['img']
						width = shape(img)[0]
						origin = Origin

						if cars[our_car]['state_info']['heading'] != None:
							sample_points,potential_values = get_sample_points(array(cars[our_car]['state_info']['pts']),angles,the_arena,cars[our_car]['state_info']['heading'])
							if mode == 'Follow_Arena_Potential_Field':
								for ang,dist in other_cars_angle_distance_list:
									indx = find_index_of_closest(-ang,angles)
									if dist > 1:
										potential_values[indx] *= (dist-1)/8.0
							steer = interpret_potential_values(potential_values)
							real_steer = cars[our_car]['runs'][run_name]['trajectory']['data']['steer'][cars[our_car]['state_info']['near_i']]
							vel = velocity[cars[our_car]['state_info']['near_i']]
							n=objects_to_angle_distance_representation(view_angles,other_cars_in_view_angle_distance_list)
							m=objects_to_angle_distance_representation(view_angles,markers_angle_distance_list)

						img_left = cars[our_car]['get_image'](run_name,'left')
						img_right = cars[our_car]['get_image'](run_name,'right')
						img = img_left.copy()
						k = animate.prepare_and_show_or_return_frame(img=img,steer=steer,motor=None,state=1,delay=1,scale=2,color_mode=cv2.COLOR_RGB2BGR,window_title='plan')
						steer = 49
						if type(img_left_previous) != bool:
							imgs = {}
							imgs['left'] = [img_left_previous,img_left]
							imgs['right'] = [img_right_previous,img_right]
							ctr = 0
							for c in range(3):
								for camera in ['left','right']:
									for t in range(2):
										solver.net.blobs['ZED_data_pool2'].data[0,ctr,:,:] = imgs[camera][t][:,:,c]
										ctr += 1
							solver.net.forward()
							steer = 100*solver.net.blobs['ip2'].data[0,9]
						img = img_left.copy()
						vel = solver.net.blobs['ip_velocity'].data[-1,:][0]
						#k = animate.prepare_and_show_or_return_frame(img=img,steer=steer,motor=20.0*vel+49,state=6,delay=1,scale=2,color_mode=cv2.COLOR_RGB2BGR,window_title='network')
						img_left_previous = img_left
						img_right_previous = img_right
						clf();xylim(0,7,0,8)
						plot(1/solver.net.blobs['target_markers'].data[-1,:],'ro-')
						plot(1/solver.net.blobs['ip_markers'].data[-1,:],'go-')
						plot(solver.net.blobs['ip_cars'].data[-1,:],'bo-')
						if k == ord('q'):
							break

					else:
						cars[our_car]['state_info']['pts'] = []



