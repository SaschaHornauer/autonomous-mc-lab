from kzpy3.utils2 import *
pythonpaths(['kzpy3','kzpy3/teg9'])
from vis2 import *
import data.utils.general
from data.utils.general import car_name_from_run_name
from arena.planner.Constants import C


def Car(N,car_name,origin,mult,markers,bair_car_data_location):
	D = {}
	D['Purpose'] = d2s(inspect.stack()[0][3],':','Car object.')
	D['car_name'] = car_name
	D['type'] = 'Car'
	D['runs'] = {}

	def _load_run(N,run_name,bair_car_data_location):
		print("load run "+run_name)
		D['runs'][run_name] = {}
		R = D['runs'][run_name]
		ts,dat = data.utils.general.get_metadata(run_name,bair_car_data_location)
		traj = lo(opj(bair_car_data_location,'meta',run_name,'traj.pkl'))
		R['ts'] = array(ts)
		R['data'] = dat
		R['traj'] = traj
		R['list_of_other_car_trajectories'] = []	
		for other_run_name in N[car_name][run_name]['other_trajectories']:
			other_car_name = car_name_from_run_name(other_run_name)
			R['list_of_other_car_trajectories'].append( [other_car_name,other_run_name] )

	for run_name in N[car_name].keys():
		if len(gg(opj(bair_car_data_location,'meta',run_name,'*.pkl'))) < 6:
			continue
		_load_run(N,run_name,bair_car_data_location)
	print("""
		Remeber time to collision.
	""")


	def _rewind():
		D['state_info'] = {}
		D['state_info']['near_i'] = 0
		D['state_info']['near_t'] = 0
		D['state_info']['near_t_prev'] = 0
		D['state_info']['pts'] = []
		D['state_info']['heading'] = None
		D['state_info']['heading_prev'] = [0,1]
		D['state_info']['relative_heading'] = 90
		D['state_info']['velocity'] = 0
	D['rewind'] = _rewind
	D['rewind']()


	def _valid_time_and_index(run_name,t):
		traj = D['traj']
		if t>traj['ts'][0] and t<traj['ts'][-1]:
			near_t = -1
			for i in range(D['near_i'],len(traj['ts'])):
				if traj['ts'][i-1]<t and traj['ts'][i]>=t:
					near_t = traj['ts'][i]
					near_i = i
					break
			if near_t > 0:
				D['near_i'] = near_i
				D['near_t'] = near_t
				return near_t,near_i
		return False,False
	D['valid_time_and_index'] = _valid_time_and_index


	def _get_image(run_name,side):
		traj = D['runs'][run_name]['trajectory']
		index = traj['data']['t_to_indx'][D['state_info']['near_t']]
		img = traj['data'][side][index]
		return img		
	D['get_image'] = _get_image


	def _load_image_and_meta_data(run_name,bair_car_data_location):
		import data.utils.general
		import data.utils.multi_preprocess_pkl_files_1_1
		bag_folders_dst_rgb1to4_path = opj(bair_car_data_location,'rgb_1to4')
		bag_folders_dst_meta_path = opj(bair_car_data_location,'meta')
		D['runs'][run_name]['trajectory']['data'] = data.utils.general.get_new_Data_dic()
		data.utils.multi_preprocess_pkl_files_1_1.multi_preprocess_pkl_files(
			D['runs'][run_name]['trajectory']['data'],
				opj(bag_folders_dst_meta_path,run_name),
				opj(bag_folders_dst_rgb1to4_path,run_name),
				print_b=True,
				load_images=False)
	D['load_image_and_meta_data'] = _load_image_and_meta_data

	print("created "+D['type']+": "+D['car_name'])
	return D





