from kzpy3.utils import *
pythonpaths(['kzpy3','kzpy3/teg9'])
from vis2 import *
import data.utils.animate as animate
from arena.planner.Constants import C
import arena.planner.Potential_Fields as Potential_Fields
import arena.planner.Cars as Cars
import arena.planner.Runs as Runs
import arena.planner.Spatial_Relations as Spatial_Relations
##############################
#
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

cars = {}
for car_name in C['car_names']:
	cars[car_name] =  Cars.Car(N,car_name,C['Origin'],C['Mult'],C['markers'])

#
###############################





current_run = Runs.Run('direct_rewrite_test_25Apr17_16h09m24s_Mr_Black',cars,an_arena,C['bair_car_data_location'])
current_run['rewind']()

car_spatial_dic,marker_spatial_dic = Spatial_Relations.setup_spatial_dics(current_run)

img = an_arena['Image']['img']
img[img>1] = 0
an_arena['show']()

timer=Timer(0)
ctr = 0


current_run['rewind']()
for t in arange(current_run['T0']+1000,current_run['Tn'],1/30.):
	Spatial_Relations.update_spatial_dics(current_run,car_spatial_dic,marker_spatial_dic,t)
	if len(current_run['our_car']['state_info']['pts']) > 0:
		xy = current_run['our_car']['state_info']['pts'][-1]
		if len(xy) > 0:
			an_arena['show']()
			an_arena['Image']['pts_plot'](xy,'r')
			for c in car_spatial_dic.keys():
				if car_spatial_dic[c]['xy'] != None:
					an_arena['Image']['pts_plot'](car_spatial_dic[c]['xy'])


			print current_run['our_car']['state_info']['heading']
			pause(0.00001)

"""		
	if len(current_run['our_car']['state_info']['pts']) > 0:
		for c in car_spatial_dic.keys():
			if car_spatial_dic[c]['xy'] != None:
				if ctr > 5:
					an_arena['show']()
					an_arena['Image']['pts_plot'](car_spatial_dic[c]['xy'])
					pause(0.00001)
					print c
					other_car = True
			else:
				other_car = False	
		
		if ctr > 5 and other_car:
			an_arena['Image']['pts_plot'](xy,'r')

			pause(0.00001)
			ctr = 0
		ctr += 1
print timer.time()
"""















