from kzpy3.utils import *
pythonpaths(['kzpy3','kzpy3/teg9'])

from vis import *
from data.markers_clockwise import markers_clockwise
import data.utils.general
from data.utils.general import car_name_from_run_name
from data.utils.general import car_colors as colors
import data.arena_display as arena_display
import data.arena
 
DISPLAY_LEFT = False
bair_car_data_location = '/media/karlzipser/ExtraDrive4/bair_car_data_new_28April2017'
markers = data.arena.Markers(range(96),4*107/100.)
if 'N' not in locals():
	print("Loading trajectory data . . .")
	N = lo(opjD('N.pkl'))



while True:
	CAR_NAME = random.choice(N.keys())
	RUN_NAME = random.choice(N[CAR_NAME].keys())
	if len(N[CAR_NAME][RUN_NAME]['other_trajectories']) > 1:
		break
		
arena_display.display_arena(N,CAR_NAME,RUN_NAME,markers,bair_car_data_location,DISPLAY_LEFT)







