


def zaccess(d,alst):
	print(zdic_to_str(d,alst))
	for a in alst:
		#print a,d
		if type(d) != dict:
			break
		d = d[sorted(d.keys())[a]]
	return d

v = zaccess(d,[1,0,0]);print v






def zlst_truncate(lst,show_ends=2):
	if show_ends == 0:
		return []
	if len(lst) > 2*show_ends:
		out_lst = lst[:show_ends] + ['...'] + lst[-show_ends:]
	else:
		out_lst = lst
	return out_lst

def zlst_to_str(lst,truncate=True,decimal_places=2,show_ends=2,depth=0,range_lst=[-2]):
	original_len = -1
	if truncate:
		original_len = len(lst)
		lst = zlst_truncate(lst,show_ends=show_ends)
	lst_str = d2n('\t'*(depth),"[")
	for i in range(len(lst)):
		e = lst[i]
		if type(e) == str:
			lst_str += e
		elif type(e) == int:
			lst_str += str(e)
		elif is_number(e):
			lst_str += str(dp(e,decimal_places))
		elif type(e) == list:
			lst_str += zlst_to_str(e,truncate=truncate,decimal_places=decimal_places,show_ends=show_ends)
		elif type(e) == dict:
			lst_str += zdic_to_str(d,range_lst,depth=depth+1)# zlst_to_str(e,truncate=truncate,decimal_places=decimal_places,show_ends=show_ends)
		else:
			lst_str += '???'
		if i < len(lst)-1:
			lst_str += ' '
	lst_str += ']'
	if original_len > 0:
		lst_str += d2n(' (len=',original_len,')')
	return lst_str





def zdic_to_str(d,range_lst,depth=0):

	dic_str_lst = []

	sorted_keys = sorted(d.keys())
	
	this_range = range_lst[0]
	
	if type(this_range) == int:
		if this_range == -1:
			this_range = [0,len(sorted_keys)]
		elif this_range == -2:
			this_range = [0,len(sorted_keys)]
			range_lst = range_lst + [-2]
		else:
			this_range = [this_range,this_range+1]

	if this_range[0] > 0:
		dic_str_lst.append(d2n('\t'*depth,'0) ...'))

	for i in range(this_range[0],this_range[1]):
		if i >= len(sorted_keys):
			return
		key = sorted_keys[i]
		value = d[key]

		dic_str_lst.append(d2n('\t'*depth,i,') ',key,':'))

		if isinstance(value,dict):
			#if max_depth_lst[0] > depth:
			if len(range_lst) > 1:
				dic_str_lst.append( zdic_to_str(value,range_lst[1:],depth=depth+1) )
			else:
				dic_str_lst.append(d2n('\t'*(depth+1),'...'))
		else:
			if type(value) == list:
				dic_str_lst.append(zlst_to_str(value,depth=depth+1,range_lst=range_lst[1:]))
			elif type(value) == np.ndarray:
				dic_str_lst.append(zlst_to_str(list(value),depth=depth+1,range_lst=range_lst[1:]))
			elif type(value) == str:
				dic_str_lst.append(d2s('\t'*(depth+1),str(value)))
			else:
				dic_str_lst.append(d2s('\t'*(depth+1),str(value),type(value)))

	if this_range[1] < len(sorted_keys):
		dic_str_lst.append(d2n('\t'*depth,'... ',len(d)-1,')'))

	dic_str = ""
	for d in dic_str_lst:
		dic_str += d + "\n"

	return dic_str


def zprint_str_lst(str_lst):
	for s in str_lst:
		if type(s) == list:
			zprint_str_lst(s)
		else:
			print(s)


a = {}
a['a'] = 'A'
a['b'] = {}
a['b']['B1'] = {}
a['b']['B1']['b1'] = 'a'
a['b']['B2'] = [1,2,3,4,5]
a['c'] = 'C'

d = {}
d['a'] = 'A'
d['b'] = {}
d['b']['B1'] = {}
d['b']['B1']['b1'] = a
d['b']['B2'] = [1,2,3,4,5]
d['c'] = 'C'



dic_str = zdic_to_str(d,[-2]);print dic_str


