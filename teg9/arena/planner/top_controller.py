from kzpy3.utils import *
pythonpaths(['kzpy3','kzpy3/teg9'])
from vis2 import *
import data.utils.animate as animate
from arena.planner.Constants import C
import arena.planner.Potential_Fields as Potential_Fields
import arena.planner.Cars as Cars


##############################
#
if False:#'N' not in locals():
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
		an_arena = a(C['Origin'],C['Mult'],C['markers'],True,1.0,1.5)
		an_arena['show']()
		the_arenas[an_arena['type']] = an_arena
		#break ##!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	the_arenas_ready = True
"""
cars = {}
for car_name in C['car_names']:
	cars[car_name] =  Cars.Car(N,car_name,C['Origin'],C['Mult'],C['markers'])
"""
#
###############################



"""

for our_car in ['Mr_Black','Mr_Silver','Mr_Yellow','Mr_Orange','Mr_Blue']:#['Mr_Yellow','Mr_Orange','Mr_Blue']:#

	for run_name in cars[our_car]['runs'].keys():


		for t in arange(T0+210,Tn,1/30.):
"""