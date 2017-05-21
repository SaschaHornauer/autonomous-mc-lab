from kzpy3.utils import *
pythonpaths(['kzpy3','kzpy3/teg9'])

from vis import *
import data.utils.general
from data.utils.general import car_name_from_run_name
from data.utils.general import car_colors as colors
import data.arena

def Car(car_name):
	D['Purpose'] = 'Car object.'
	D = {}
	D['car_name'] = car_name
	D['run_name'] = ''
	D['left_trajectory'] = None
	D['right_trajectory'] = None
	D['pfield'] = None
	D['xy'] = None
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