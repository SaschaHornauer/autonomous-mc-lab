from kzpy3.utils import *
pythonpaths(['kzpy3','kzpy3/teg9'])
from vis2 import *
import data.utils.animate as animate
from arena.planner.Constants import C
import arena.planner.Potential_Fields as Potential_Fields
import arena.planner.Cars as Cars
import arena.planner.Runs as Runs
import arena.planner.Spatial_Relations as Spatial_Relations


if 'N' not in locals():
	print("Loading trajectory data . . .")
	N = lo(C['trajectory_data_location'])

if 'the_arenas_ready' not in locals():
	print("Creating arenas . . .")
	args = []
	arenas_tmp_lst = [Potential_Fields.Direct_Arena_Potential_Field,
		Potential_Fields.Play_Arena_Potential_Field,
		Potential_Fields.Follow_Arena_Potential_Field,
		Potential_Fields.Furtive_Arena_Potential_Field]
	the_arenas = {}
	for a in arenas_tmp_lst:
		an_arena = a(C['Origin'],C['Mult'],C['markers'],False,1.0,1.5)
		the_arenas[an_arena['type']] = an_arena
		break
	the_arenas_ready = True
	img = an_arena['Image']['img'] #!!!!!!!!!! TEMP
	img[img>1] = 0



cars = {}
for car_name in C['car_names']:
	cars[car_name] =  Cars.Car(N,car_name,C['Origin'],C['Mult'],C['markers'])




#current_run = Runs.Run('direct_rewrite_test_25Apr17_16h09m24s_Mr_Black',cars,an_arena,C['bair_car_data_location'])
current_run = Runs.Run('direct_Fern_aruco_1_14Apr17_20h24m03s_Mr_Yellow',cars,an_arena,C['bair_car_data_location'])





T_OFFSET_VALUE = 0

ctr_timer = Timer(0)
timer = Timer(5)
ctr = 0

current_run['rewind']()
for t in arange(current_run['T0']+T_OFFSET_VALUE,current_run['Tn'],1/30.):
	if timer.check():
		pd2s(dp(ctr/ctr_timer.time()/30.0),dp(t-current_run['T0']),dp(100.0*(t-current_run['T0'])/(current_run['Tn']-current_run['T0'])),'%')
		timer.reset()
	if Spatial_Relations.update_spatial_dics(current_run,current_run['car_spatial_dic'],current_run['marker_spatial_dic'],t):
		heading = current_run['our_car']['state_info']['heading']
		if heading != None:
			car_angle_dist_view = Spatial_Relations.get_angle_distance_view(current_run,'car_spatial_dic')
			if True: #len(car_angle_dist_view) > 0:
				pd2s(dp(current_run['our_car']['state_info']['relative_heading']))
				marker_angle_dist_view = Spatial_Relations.get_angle_distance_view(current_run,'marker_spatial_dic')
				Runs.show_arena_with_cars(current_run,an_arena,t)
				pause(0.0001)
				potential_values = Spatial_Relations.get_sample_points(current_run['our_car']['state_info']['pts'],
					C['sensor_angles'],an_arena,heading)
				ctr += 1
				figure('view')
				clf()
				xylim(0,7,0,2)
				plot(car_angle_dist_view,'r.-')
				plot(marker_angle_dist_view,'b.-')
				plot(array(potential_values)*2.5,'ko-')
				
	else:
		continue
		#clf()
		#current_run['the_arena']['show']()
		#pause(0.0001)
			