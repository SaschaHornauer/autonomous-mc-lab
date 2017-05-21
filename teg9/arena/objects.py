from kzpy3.utils import *
pythonpaths(['kzpy3','kzpy3/teg9'])

from vis import *
import data.utils.general
from data.utils.general import car_name_from_run_name
from data.utils.general import car_colors as colors
#import data.arena



def Markers(markers_clockwise,radius):
	D = {}
	D['Purpose'] = 'Markers for the aruco arena.'
	D['clockwise'] = markers_clockwise
	D['ids_all'] = []
	D['angles_dic'] = {}
	D['angles'] = 2*np.pi*np.arange(len(markers_clockwise))/(1.0*len(markers_clockwise))
	D['xy'] = []
	for i in range(len(markers_clockwise)):
		a = D['angles'][i]
		D['angles_dic'][markers_clockwise[i]] = a
		x = radius*np.sin(a)
		y = radius*np.cos(a)
		D['xy'].append([x,y])
	D['xy_dic'] = {}
	assert(len(markers_clockwise) == len(D['xy']))
	def _cv2_draw(img):
		for j in range(len(D['clockwise'])):
			m = D['clockwise'][j]
			xy = D['xy'][j]
			D['xy_dic'][m] = xy
			c = (255,0,0)
			xp,yp = img['floats_to_pixels'](img,xy)
			cv2.circle(img['img'],(xp,yp),4,c,-1)
	D['cv2_draw'] = _cv2_draw
	return D


"""


def Car(N,car_name):
	D['Purpose'] = 'Car object.'
	D = {}
	D['car_name'] = car_name
	D['runs'] = {}
	D['left_trajectory'] = None
	D['right_trajectory'] = None
	D['pfield'] = None
	D['xy'] = None

	for run_name in N[car_name].keys():
		D['runs'][run_name] = {}
		R = D['runs'][run_name]
		R['trajectory'] = N[car_name][run_name]['self_trajectory']
		R['list_of_other_car_trajectories'] = []
		for ot in N[car_name][run_name]['other_trajectories']:
			other_run_name = ot['run_name']
			other_car_name = car_name_from_run_name(other_run_name)
			R['list_of_other_car_trajectories'].append( N[other_car_name][other_run_name]['self_trajectory'] )
		

	return D









def Gaussian_2D(width):
	return makeGaussian(width,width/3.0)



def Potential_Field(xy_sizes,origin,mult)):
	D['Purpose'] = 'Potential field for path planning.'
	D['Image'] = Image(xyz_sizes,origin,mult,data_type=np.float)
	D['previous_additions'] = []
	D['sub_add'] = _sub_add
	return D

def _sub_add(D,additions):
	for p in D['previous_additions']:
		# subtract them
	for p in additions:
		# add them
	D['previous_additions'] = additions





def Arena_Potential_Field(xy_sizes,origin,mult,markers)):
	D = Potential_Field(xy_sizes,origin,mult)
	D['Purpose'] = d2s('Potential field specific for arena.',D[Purpose])
	gau_marker = Gaussian_2D()
	D['sub_add']([g1,g2,g3])
	return D

"""

