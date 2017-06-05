





def relation_to_other_object(our_heading,xy_our,xy_other,view_angle):
	in_view = False
	angle_to_other = angle_clockwise(our_heading,array(xy_other)-array(xy_our))
	if angle_to_other > 360-view_angle:
		angle_to_other = angle_to_other-360
	distance_to_other = length(xy_other-xy_our)
	if angle_to_other > -view_angle and angle_to_other < view_angle:
		in_view = True
	return angle_to_other,distance_to_other,in_view



def objects_to_angle_distance_representation(reference_angles,other_angle_distance_list):
	m = array(reference_angles)*0.0
	if len(reference_angles) > len(other_angle_distance_list):
		for object_angle,object_distance in other_angle_distance_list:
			indx = find_index_of_closest(object_angle,reference_angles)
			if m[indx] < 1/object_distance:
				m[indx] = 1/object_distance
	else:
		other_angle_distance_array = array(other_angle_distance_list)
		other_angles = other_angle_distance_array[:,0]
		other_distances = other_angle_distance_array[:,1]
		for i in range(len(reference_angles)):
			indx = find_index_of_closest(reference_angles[i],other_angles)
			m[i] = 1/other_distances[indx]
	return m





	xy_our = cars[our_car]['report_camera_positions'](run_name,t)

	other_cars_xy_list = []
	other_cars_in_view_xy_list = []
	other_cars_angle_distance_list = []
	other_cars_in_view_angle_distance_list = []

	markers_xy_list = []
	markers_angle_distance_list = []
	markers_in_view_xy_list = []
	markers_in_view_angle_distance_list = []

	no_cars_in_view = True
	no_markers_in_view = True

	if len(xy_our) > 0:
		xy_our = array(xy_our)
		our_heading = cars[our_car]['state_info']['heading']
		for l in list_of_other_car_trajectories:
			other_car_name = l[0]
			other_car_run_name = l[1]
			xy_other = cars[other_car_name]['report_camera_positions'](other_car_run_name,t)
			if len(xy_other) > 0:
				if our_heading != None:
					angle_to_other,distance_to_other,in_view = relation_to_other_object(our_heading,xy_our,xy_other,view_angle)
					other_cars_angle_distance_list.append([angle_to_other,distance_to_other])
					other_cars_xy_list.append(xy_other)
					other_cars_angle_distance_list.append([angle_to_other,distance_to_other])
					if in_view:
						no_cars_in_view = False
						other_cars_in_view_angle_distance_list.append([angle_to_other,distance_to_other])
						other_cars_in_view_xy_list.append(xy_other)
		for xy_other in markers['xy']:
			if len(xy_other) > 0:
				if our_heading != None:
					angle_to_other,distance_to_other,in_view = relation_to_other_object(our_heading,xy_our,xy_other,view_angle)
					markers_angle_distance_list.append([angle_to_other,distance_to_other])
					markers_xy_list.append(xy_other)
					markers_angle_distance_list.append([angle_to_other,distance_to_other])
					markers_xy_list.append(xy_other)
					if in_view:
						no_markers_in_view = False
						markers_in_view_angle_distance_list.append([angle_to_other,distance_to_other])
						markers_in_view_xy_list.append(xy_other)









velocity = (cars[our_car]['runs'][run_name]['trajectory']['left']['t_vel']+cars[our_car]['runs'][run_name]['trajectory']['right']['t_vel'])/2.0


	def _get_sample_points(pts,angles,pfield,heading):
	    sample_points = []
	    potential_values = []
	    heading *= 0.5 # 50 cm, about the length of the car
	    for the_arena in angles:
	        sample_points.append( rotatePoint([0,0],heading,the_arena) )
	    for k in range(len(sample_points)):
	        f = sample_points[k]
	    for sp in sample_points:
	    	if GRAPHICS:
	    		pfield['Image']['plot_pts'](array(sp)+array(pts[-1,:]),'g')
	        pix = pfield['Image']['floats_to_pixels']([sp[0]+pts[-1,0],sp[1]+pts[-1,1]])
	        potential_values.append(pfield['Image']['img'][pix[0],pix[1]])
	    return sample_points,potential_values

	def _interpret_potential_values(potential_values):
		min_potential_index = potential_values.index(min(potential_values))
		max_potential_index = potential_values.index(max(potential_values))
		middle_index = int(len(potential_values)/2)
		potential_values = array(potential_values)
		pmin = potential_values.min()
		pmax = potential_values.max()
		potential_values = z2o(potential_values) * pmax
		if GRAPHICS:
			figure(9);plot(potential_values,'bo-')
		d = 99.0/(1.0*len(potential_values)-1)
		steer_angles = np.floor(99-arange(0,100,d))
		p = min(pmax/0.8,1.0)
		steer = int((p*steer_angles[min_potential_index]+(1-p)*49.0))
		return steer


