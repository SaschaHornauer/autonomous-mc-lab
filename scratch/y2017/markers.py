from kzpy3.vis import *
from kzpy3.data_analysis.Angle_Dict_Creator import get_angles_and_distance

def plot_it(angle1,distance1,angle2,distance2):


	xc = distance1 * np.sin(angle1)
	yc = distance1 * np.cos(angle1)

	plot([0,xc],[0,yc])

	xd = distance2 * np.sin(angle2)
	yd = distance2 * np.cos(angle2)

	plot([xc,xd+xc],[yc,yd+yc])

	circ=plt.Circle((xd+xc,yd+yc),distance2,fill=False)

	ax.add_patch(circ)
	pause(0.03)

marker_ids_all = []
"""
for i in range(0,2300):
	img = imread('/home/karlzipser/Desktop/temp2_/'+str(i)+'.png' )
	#mi(img,2)
	angles_to_center, angles_surfaces, distances_marker, markers = get_angles_and_distance(img) 

	#fig=plt.figure(1)
	#plt.clf()
	#ax=fig.add_subplot(1,1,1)
	#xlim(-10,10)
	#ylim(-10,10)

	distance2 = 4*107/100.

	marker_ids = angles_to_center.keys()

	for m in marker_ids:

		angle1 = angles_to_center[m]
		distance1 = distances_marker[m]
		angle2 = angles_surfaces[m]
		if distance1 < 2:
			marker_ids_all.append(m)
		if distance1 < 2:
			pass #plot_it(angle1,distance1,angle2,distance2)

	#raw_input('>')
		
if False:
	seen = {}
	new = []
	for i in range(3500,len(marker_ids_all)):
		m = marker_ids_all[i]
		if m in seen:
			pass
		else:
			new.append(m)
			seen[m] = True
"""
marker_angles_dic = {}
marker_angles = 2*np.pi*np.arange(len(markers_clockwise))/(1.0*len(markers_clockwise))
marker_xys = []
for i in range(len(markers_clockwise)):
	a = marker_angles[i]
	marker_angles_dic[markers_clockwise[i]] = a
	x = 4*107/100.*np.sin(a)
	y = 4*107/100.*np.cos(a)
	marker_xys.append([x,y])
#marker_xys = array(marker_xys)

markers_xy_dic = {}
assert(len(markers_clockwise) == len(marker_xys))
for i in range(len(markers_clockwise)):
	m = markers_clockwise[i]
	xy = marker_xys[i]
	markers_xy_dic[m] = xy
	plot(xy[0],xy[1],'bo-')
	plt.text(xy[0],xy[1],str(m),fontsize=6)


#plot(marker_xys[:,0],marker_xys[:,1],'o')





def plot_it2(angle1,distance1,angle2,distance2,xy):


	#xc = distance1 * np.sin(angle1)
	#yc = distance1 * np.cos(angle1)

	#plot([0,xc],[0,yc])

	xd = distance1 * np.sin(angle2)
	yd = distance1 * np.cos(angle2)

	plot([xy[0],xd+xy[0]],[xy[1],yd+xy[1]])

	#circ=plt.Circle((xd+xc,yd+yc),distance2,fill=False)

	#ax.add_patch(circ)
	pause(0.001)


for i in range(0,2300):
	try:
		img = imread('/home/karlzipser/Desktop/temp2_/'+str(i)+'.png' )
		#mi(img,2)
		angles_to_center, angles_surfaces, distances_marker, markers = get_angles_and_distance(img) 

		#fig=plt.figure(1)
		#plt.clf()
		#ax=fig.add_subplot(1,1,1)
		#xlim(-10,10)
		#ylim(-10,10)

		distance2 = 4*107/100.

		marker_ids = angles_to_center.keys()
		xlim(-5,5);ylim(-5,5)
		for m in marker_ids:
			print m
			xy = markers_xy_dic[m]
			angle1 = angles_to_center[m]
			distance1 = distances_marker[m]
			angle2 = marker_angles_dic[m]-angles_surfaces[m]
			if distance1 < 2:
				plot_it2(angle1,distance1,angle2,distance2,xy)
	except:
		pass

