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

for i in range(300,2300):
	img = imread('/home/karlzipser/Desktop/temp2_/'+str(i)+'.png' )
	mi(img,2)
	angles_to_center, angles_surfaces, distances_marker, markers = get_angles_and_distance(img) 

	fig=plt.figure(1)
	plt.clf()
	ax=fig.add_subplot(1,1,1)
	xlim(-10,10)
	ylim(-10,10)

	distance2 = 4*107/100.

	marker_ids = angles_to_center.keys()

	for m in marker_ids:
			angle1 = angles_to_center[m]
			distance1 = distances_marker[m]
			angle2 = angles_surfaces[m]
			if distance1 < 2:
				plot_it(angle1,distance1,angle2,distance2)

	#raw_input('>')
		