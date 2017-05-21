from kzpy3.utils import *
pythonpaths(['kzpy3','kzpy3/teg9'])

import inspect


from vis import *
import data.utils.general
from data.utils.general import car_name_from_run_name
from data.utils.general import car_colors as colors



def Markers(markers_clockwise,radius):
	D = {}
	D['Purpose'] = d2s(inspect.stack()[0][3],':','Markers for the aruco arena.')
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








def Potential_Field(xy_sizes,origin,mult):
	D = {}
	D['Purpose'] = d2s(inspect.stack()[0][3],':','Potential field for path planning.')
	D['Image'] = Image(xy_sizes,origin,mult,data_type=np.float)
	D['previous_additions'] = []
	def _sub_add(additions):
		for p in D['previous_additions']:
			isub(p[0],D['Image']['img'],p[1])
		D['previous_additions'] = []
		for a in additions:
			D['add'](a[0],a[1])
	def _add(addition,xy):
		xy_pixels = D['Image']['floats_to_pixels'](xy)
		iadd(addition,D['Image']['img'],xy_pixels)
		D['previous_additions'].append([addition,xy_pixels])
	D['sub_add'] = _sub_add
	D['add'] = _add
	return D




def Arena_Potential_Field(origin,mult,markers):
	xy_sizes = [2*origin,2*origin]
	D = Potential_Field(xy_sizes,origin,mult)
	D['Purpose'] = d2f('\n',d2s(inspect.stack()[0][3],':','Potential field specific for arena.'),D['Purpose'])
	gau_marker = Gaussian_2D(mult)
	gau_s = Gaussian_2D(0.24*mult)
	gau_center = Gaussian_2D(6*mult)
	gau_follow = Gaussian_2D(12*mult)
	gau_car = Gaussian_2D(6*mult)
	for xy in markers['xy']:
		D['add'](gau_marker,xy)
		D['add'](-1.0*gau_s,xy)
		D['add'](-1.0*gau_marker,0.75*(array(xy)))
		D['add'](2*gau_marker,1.05*(array(xy)))
		D['add'](3*gau_marker,1.1*(array(xy)))
		D['add'](4*gau_marker,1.15*(array(xy)))
	D['add'](4*gau_center,[0,0])
	D['previous_additions'] = []
	def _test(iterations=1,Graphics=True):
		timer = Timer(0)
		ctr = 0
		for i in range(iterations):
			print (dp(timer.time(),2),ctr)
			for xy in markers['xy']:
				ctr += 1
				D['sub_add']([[-15*gau_follow,0.75*(array(xy))],[10*gau_marker,0.75*(array(xy))]])
				if Graphics:
					img = D['Image']['img']
					width = shape(img)[0]
					mi(img[width/2-origin/2:width/2+origin/2,width/2-origin/2:width/2+origin/2],1)
					figure(2);clf();plot(a['Image']['img'][Origin,:],'o-');pause(0.0001)
		print timer.time()
		print ctr
	D['test'] = _test
	return D










def Car(N,car_name,origin,mult,markers):
	D = {}
	D['Purpose'] = d2s(inspect.stack()[0][3],':','Car object.')
	D['car_name'] = car_name
	D['potential_field'] = Arena_Potential_Field(origin,mult,markers)
	D['xy'] = [0,0]
	D['runs'] = {}
	for run_name in N[car_name].keys():
		D['runs'][run_name] = {}
		R = D['runs'][run_name]
		R['trajectory'] = N[car_name][run_name]['self_trajectory']
		R['list_of_other_car_trajectories'] = []
		for ot in N[car_name][run_name]['other_trajectories']:
			other_run_name = ot['run_name']
			other_car_name = car_name_from_run_name(other_run_name)
			R['list_of_other_car_trajectories'].append( [other_car_name,other_run_name] )
	return D
	def _report_position(t):

		return xy, xy_left, xy_right, confidence
	D['report_position'] = _report_position
	D['positions'] = {}


if True:
	from arena.markers_clockwise import markers_clockwise
	markers = Markers(markers_clockwise,4*107/100.)
	Origin = int(2*1000/300.*300)# / 5)
	Mult = 1000/300.*50# / 5
	#a = Arena_Potential_Field(Origin,Mult,markers)
	#figure(2);clf();plot(a['Image']['img'][Origin,:],'o-')
	#a['test']()
	c = Car(N,'Mr_Orange',Origin,Mult,markers)


