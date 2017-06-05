from kzpy3.utils import *
pythonpaths(['kzpy3','kzpy3/teg9'])
from vis import *
import data.utils.animate as animate
import arena.planner.Markers as Markers
import arena.planner.Potential_Fields as Potential_Fields
import arena.planner.Cars as Cars

#######################################
#
#bair_car_data_location = '/media/karlzipser/bair_car_data_new_bkp1/bair_car_data_new_28April2017'
#bair_car_data_location = '/media/karlzipser/SSD_2TB/bair_car_data_new_28April2017'
bair_car_data_location = '/Volumes/SSD_2TB/bair_car_data_new_28April2017'
#bair_car_data_location = '/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017'
#bair_car_data_location = opjD('bair_car_data_new_28April2017')

trajectory_data_location = opjD('N.pkl')
for p in [bair_car_data_location,trajectory_data_location]:
	assert(len(gg(p))) > 0
#
#######################################
#
angles = -arange(-45,46,9)
view_angle = 35
view_angles = arange(-view_angle,view_angle+1,10)

DISPLAY_LEFT = True
GRAPHICS = True
GRAPHICS2 = True
markers = Markers.Markers(Markers.markers_clockwise,4*107/100.)
Origin = int(2*1000/300.*300 / 5)
Mult = 1000/300.*50 / 5
#
#######################################

##############################
#
if 'N' not in locals():
	print("Loading trajectory data . . .")
	N = lo(trajectory_data_location)

if 'the_arenas' not in locals():
	print("Creating arenas . . .")
	arenas_tmp_lst = [Potential_Fields.Direct_Arena_Potential_Field(Origin,Mult,markers),
		Potential_Fields.Play_Arena_Potential_Field(Origin,Mult,markers),
		Potential_Fields.Follow_Arena_Potential_Field(Origin,Mult,markers),
		Potential_Fields.Furtive_Arena_Potential_Field(Origin,Mult,markers)]
	the_arenas = {}
	for a in arenas_tmp_lst:
		the_arenas[a['type']] = a
		break

cars = {}
for car_name in ['Mr_Black','Mr_Silver','Mr_Yellow','Mr_Orange','Mr_Blue']:
	cars[car_name] =  Cars.Car(N,car_name,Origin,Mult,markers)
#
###############################





for our_car in ['Mr_Black','Mr_Silver','Mr_Yellow','Mr_Orange','Mr_Blue']:#['Mr_Yellow','Mr_Orange','Mr_Blue']:#

	for run_name in cars[our_car]['runs'].keys():


		for t in arange(T0+210,Tn,1/30.):
