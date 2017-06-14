from kzpy3.utils2 import *
pythonpaths(['kzpy3','kzpy3/teg9'])
from vis2 import *
import data.utils.animate as animate
from arena.planner.Constants import C
import arena.planner.Potential_Fields as Potential_Fields
import arena.planner.Cars as Cars
import arena.planner.Runs as Runs
import arena.planner.Spatial_Relations as Spatial_Relations
import data.utils.general
# clockwise = 270° relative angle, to wall = 0°, counter-clockwise = 90°

# Add potential measures and relative heading value
GRAPHICS = True
SAVE_DATA = False
T_OFFSET_VALUE = 28
TIME_STEP = 1/30.0

print_stars();print('*')
print(d2s('T_OFFSET_VALUE =',T_OFFSET_VALUE))
print(d2s('TIME_STEP =',TIME_STEP))
pause(1.0);
print('*');print_stars()


if 'N' not in locals():
	print("Loading trajectory data . . .")
	N = lo(C['trajectory_data_location'])

if 'the_arenas_ready' not in locals():
	print("Creating arenas . . .")
	arenas_tmp_lst = [Potential_Fields.Direct_Arena_Potential_Field]#,
		#Potential_Fields.Follow_Arena_Potential_Field,
		#Potential_Fields.Play_Arena_Potential_Field,
		#Potential_Fields.Furtive_Arena_Potential_Field]
	the_arenas = {}
	for a in arenas_tmp_lst:
		an_arena = a(C['Origin'],C['Mult'],C['markers'],False,1.0,1.5)
		the_arenas[an_arena['type']] = an_arena
	del arenas_tmp_lst
	the_arenas_ready = True

if 'the_cars_ready' not in locals():
	print("Loading cars . . .")
	cars = {}
	for car_name in C['car_names']:
		cars[car_name] =  Cars.Car(N,car_name,C['Origin'],C['Mult'],C['markers'],C['bair_car_data_location'])
	the_cars_ready = True

if SAVE_DATA:
	unix('mkdir -p '+opjD('output_data'))


for car_name in [C['car_names'][0]]:

	for run_name in cars[car_name]['runs'].keys():
		output_data = {}
		try:

			current_run = Runs.Run(run_name,cars,an_arena,C['bair_car_data_location'])

			if GRAPHICS:
				images = data.utils.general.get_bag_pkl_images(current_run['run_name'],C['bair_car_data_location'])

			output_data[run_name] = {}
			output_name = opjD('output_data',run_name+'.output_data.pkl')

			if SAVE_DATA:
				if len(gg(output_name)) > 0:
					print(output_name+' exists, continuing.')
					continue

			for k in the_arenas:

				an_arena = the_arenas[k]
				mode = an_arena['type']
				output_data[run_name][mode] = {}
				output_data[run_name][mode]['potential_values'] = []
				output_data[run_name][mode]['steer'] = []
				output_data[run_name][mode]['velocity'] = []
				output_data[run_name][mode]['near_t'] = []
				output_data[run_name][mode]['near_i'] = []
				output_data[run_name][mode]['marker_inverse_distances'] = []
				output_data[run_name][mode]['other_car_inverse_distances'] = []
				output_data[run_name][mode]['relative_heading'] = []
				output_data[run_name][mode]['current_direction'] = []




				ctr_timer = Timer(0)
				timer = Timer(5)
				ctr = 0
				wise_delta = 20
				current_run['rewind']()
				clockwise = True
				for t in arange(current_run['T0']+T_OFFSET_VALUE,current_run['Tn'],TIME_STEP):
					if timer.check():
						pd2s(dp(ctr/ctr_timer.time()/30.0),'Hz',dp(t-current_run['T0']),'seconds in',dp(100.0*(t-current_run['T0'])/(current_run['Tn']-current_run['T0'])),'%')
						timer.reset()
					if Spatial_Relations.update_spatial_dics(current_run,current_run['car_spatial_dic'],current_run['marker_spatial_dic'],t):
						heading = current_run['our_car']['current_heading']()
						if heading != None:
							car_angle_dist_view = Spatial_Relations.get_angle_distance_view(current_run,'car_spatial_dic')

							if len(car_angle_dist_view) > 0:

								other_cars_in_view_xy_list = []
								for c in current_run['car_spatial_dic'].keys():
									if current_run['car_spatial_dic'][c]['in_view']:
										other_cars_in_view_xy_list.append(current_run['car_spatial_dic'][c]['xy'])


								an_arena['other_cars'](other_cars_in_view_xy_list,an_arena['type'],current_run['our_car']['current_xy']())
								
								marker_angle_dist_view = Spatial_Relations.get_angle_distance_view(current_run,'marker_spatial_dic',are_markers=True)
								
								potential_values = Spatial_Relations.get_sample_points(current_run['our_car']['current_xy'](),
									C['sensor_angles'],an_arena,heading)
								
								if an_arena['type'] == 'Follow_Arena_Potential_Field':
									dists = []
									for i in range(len(car_angle_dist_view)):
										inv_dist = car_angle_dist_view[i]
										dist = 1/(0.00001+inv_dist)
										dists.append(dist)
									if len(dists) > 0:
										if min(dists) <= 1:
											pass
										else:
											for i in range(len(dists)):
												dist = dists[i]
												if dist >= 1 and dist < 8:
													potential_values[i] *= 1.0/(9.0-dist)
								else:
									potential_values = 3*array(potential_values) # ????


								relative_heading = angle_clockwise(current_run['our_car']['current_heading'](),current_run['our_car']['current_xy']())
								#relative_heading = current_run['our_car']['current_relative_heading']() # this is problematic because of smoothing, thus recalculate now
								if relative_heading > 360-wise_delta or relative_heading <= wise_delta or length(current_run['our_car']['current_xy']()) < 1.0:
									direction = 'in'
								elif relative_heading > wise_delta and relative_heading <= 180-wise_delta:
									direction = 'counter-clockwise'
								elif relative_heading > 180+wise_delta and relative_heading <= 360-wise_delta:
									direction = 'clockwise'
								else:
									direction = 'out'

								print(direction,int(relative_heading),int(relative_heading),int(360-wise_delta),int(wise_delta))


								if direction == 'clockwise':
									if not clockwise:
										clockwise = True
								elif direction == 'counter-clockwise':
									if clockwise:
										clockwise = False

								clock_potential_values = z2o(arange(len(potential_values)))
								if clockwise:
									clock_potential_values = 1 - clock_potential_values
								clock_potential_values *= 5.0*length(0.5+current_run['our_car']['current_xy']())/C['Marker_Radius'] #???

								if relative_heading >= 0 and relative_heading < 90:
									clock_potential_values *= abs(95-relative_heading)/90.0
								elif relative_heading >= 270 and relative_heading <= 360:
									clock_potential_values *= abs(relative_heading-265)/90.0
								else:
									clock_potential_values *= 0





								ctr += 1




								new_steer = Spatial_Relations.interpret_potential_values(list(potential_values+clock_potential_values))





								output_data[run_name][mode]['marker_inverse_distances'].append(marker_angle_dist_view)
								output_data[run_name][mode]['other_car_inverse_distances'].append(car_angle_dist_view)
								output_data[run_name][mode]['potential_values'].append(potential_values)
								output_data[run_name][mode]['steer'].append(new_steer)
								output_data[run_name][mode]['near_t'].append(current_run['our_car']['near_t'])
								output_data[run_name][mode]['near_i'].append(current_run['our_car']['near_i'])
								output_data[run_name][mode]['velocity'].append(current_run['our_car']['current_velocity']())

								if GRAPHICS:
									Runs.show_arena_with_cars(current_run,an_arena,t)
									figure('view')
									clf()
									xylim(0,11,0,2)
									plot(car_angle_dist_view,'y.-')
									plot(marker_angle_dist_view,'b.-')
									plot(potential_values,'r.-')
									plot(clock_potential_values,'gx-')
									#plot(potential_values+clock_potential_values,'ko-')
									plt.title(d2s(new_steer))
									pause(0.0001)
									mci(images['left'][current_run['our_car']['near_t']],delay=1,title='images',scale=2.0)
						else:
							continue

			if SAVE_DATA:
				so(output_data,output_name)
				print('saved '+output_name)
				print(output_data[run_name].keys())
				print_stars()


		except Exception as e:
			print("********** Exception ***********************")
			print(e.message, e.args)


