
from kzpy3.vis import *
import rospy
import rosbag
import cv2
from cv_bridge import CvBridge,CvBridgeError
import threading
import kzpy3.teg9.data.animate as animate

bridge = CvBridge()

image_topics = ['left_image','right_image']
single_value_topics = ['steer','state','motor','encoder','GPS2_lat']
vector3_topics = ['acc','gyro','gyro_heading']#,'gps']
camera_sides = ['left','right']




def multi_preprocess_bagfiles(A,meta_path,bag_folder_path,bagfile_range=[]):

    A['meta'] = load_obj(meta_path)

    bag_files = sorted(gg(opj(bag_folder_path,'*.bag')))
    if len(bagfile_range) > 0:
        bag_files = bag_files[bagfile_range[0]:bagfile_range[1]]
    A['images'] = []
    threading.Thread(target=multi_preprocess_bagfiles_thread,args=[A,bag_files]).start()




def multi_preprocess_bagfiles_thread(A,bag_files):
    A['steer_previous'] = 49
    A['motor_previous'] = 49
    for b in bag_files:
        if A['STOP_LOADER_THREAD']:
            print('Stopping multi_preprocess_thread.')
            break
        preprocess_bagfiles(A,b)




def preprocess_bagfiles(A,path):
    timer = Timer(0)
    
    for topic in image_topics + single_value_topics:
        if topic not in A:
            A[topic] = []
    for topic in vector3_topics:
        if topic+'_x' not in A:
            A[topic+'_x'] = []
            A[topic+'_y'] = []
            A[topic+'_z'] = []

    """
    -2<acc_z<3,0.5
    -2.5<acc_x<3,0.5
    6<acc_y<13,9.8

    -50 gyro_x 50
    -40, gyro_y 40
    -30, gyro_z 30
    """


    if True:#try:
        cprint('Loading bagfile '+path,'yellow')

        bag = rosbag.Bag(path)

        color_mode = "rgb8"

        for s in ['left']:
            for m in bag.read_messages(topics=['/bair_car/zed/'+s+'/image_rect_color']):
                t = round(m.timestamp.to_time(),3)
                if A['t_previous'] > 0:            
                    if s == 'left':
                        A['left_deltas'].append([t,t-A['t_previous']])
                A['t_previous'] = t
                
                A['images'].append(bridge.imgmsg_to_cv2(m[1],color_mode))

                if t not in A['meta']:
                    print(d2s(t,"not in A['meta']"))
                try:
                    if A['SMOOTHING']:
                        A['steer'].append((A['meta'][t]['steer']+A['steer_previous'])/2.0)
                        A['motor'].append((A['meta'][t]['motor']+A['motor_previous'])/2.0)
                        A['state'].append(A['meta'][t]['state'])
                        A['steer_previous'] = A['steer'][-1]
                        A['motor_previous'] = A['motor'][-1]
                    else:
                        A['steer'].append(A['meta'][t]['steer'])
                        A['state'].append(A['meta'][t]['state'])
                        A['motor'].append(A['meta'][t]['motor'])
                except:
                    A['steer'].append(0)
                    A['state'].append(0)
                    A['motor'].append(0)
    #except Exception as e:
    #    print e.message, e.args
    print(d2s('Done in',timer.time(),'seconds'))





def multi_preprocess_pkl_files(A,meta_path,rgb_1to4_path):
    for topic in image_topics + single_value_topics:
        if topic not in A:
            A[topic] = []
    for topic in vector3_topics:
        if topic+'_x' not in A:
            A[topic+'_x'] = []
            A[topic+'_y'] = []
            A[topic+'_z'] = []
    A['meta'] = load_obj(opj(meta_path,'left_image_bound_to_data.pkl'))
    steer_previous = 49
    motor_previous = 49
    bag_pkls = sgg(opj(rgb_1to4_path,'*.bag.pkl'))
    for b in bag_pkls:
        print b
        o = load_obj(b)
        ts = sorted(o['left'].keys())
        for t in ts:
            A['images'].append(o['left'][t])
            try:
                if A['SMOOTHING']:
                    A['steer'].append((A['meta'][t]['steer']+steer_previous)/2.0)
                    A['motor'].append((A['meta'][t]['motor']+motor_previous)/2.0)
                    A['state'].append(A['meta'][t]['state'])
                    steer_previous = A['steer'][-1]
                    motor_previous = A['motor'][-1]
                else:
                    A['steer'].append(A['meta'][t]['steer'])
                    A['state'].append(A['meta'][t]['state'])
                    A['motor'].append(A['meta'][t]['motor'])
            except:
                A['steer'].append(0)
                A['state'].append(0)
                A['motor'].append(0)
            try:
                A['acc_x'].append(A['meta'][t]['acc'][0])
                A['acc_y'].append(A['meta'][t]['acc'][1])
                A['acc_z'].append(A['meta'][t]['acc'][2])

            except:
                A['acc_x'].append(0)
                A['acc_y'].append(1)
                A['acc_z'].append(2)
            try:
                A['gyro_x'].append(A['meta'][t]['gyro'][0])
                A['gyro_y'].append(A['meta'][t]['gyro'][1])
                A['gyro_z'].append(A['meta'][t]['gyro'][2])

            except:
                A['gyro_x'].append(0)
                A['gyro_y'].append(1)
                A['gyro_z'].append(2)
            try:
                A['encoder'].append(A['meta'][t]['encoder'])

            except:
                A['encoder'].append(0)

    A['acc_xz_dst'] = sqrt(array(A['acc_x'])**2 + array(A['acc_z'])**2)
    A['collisions'] = 0*array(A['steer'])






def get_new_A(_=None):
    A = {}
    A['STOP_LOADER_THREAD'] = False
    A['STOP_ANIMATOR_THREAD'] = False
    A['STOP_GRAPH_THREAD'] = False
    A['d_indx'] = 1.0
    A['current_img_index'] = -A['d_indx']
    A['t_previous'] = 0
    A['left_deltas'] = []
    A['scale'] = 4.0
    A['delay'] = 33
    A['steer'] = []
    A['state'] = []
    A['SMOOTHING'] = True
    A['motor'] = []
    A['images'] = []
    A['meta'] = None
    A['color_mode'] = cv2.COLOR_RGB2BGR
    A['save_start_index'] = 6700
    A['save_stop_index'] = 7024
    return A


#A = {}
#bag_or_pkl = "pkl"  
#data_path = opjD('bair_car_data_new')
run_name = "direct_rewrite_test_25Apr17_12h40m27s_Mr_Black"




from kzpy3.vis import *
from kzpy3.data_analysis.Angle_Dict_Creator import get_angles_and_distance
from kzpy3.data_analysis.markers_clockwise import markers_clockwise

def main():
    A = {}
    
    bag_or_pkl = sys.argv[1]
    if bag_or_pkl == 'bag':
        print('Working with bag files')
    elif bag_or_pkl == 'pkl':
        print('Working with pkl file')
    else:
        print("sys.argv[1] must be 'bag' or 'pkl'.")
        return
    data_path = sys.argv[2]
    if len(gg(data_path)) != 1:
        print("if len(gg(meta_path)) != 1:")
        return
    run_name = sys.argv[3]

    if len(sys.argv) > 4:
        alt_bagfolder_path = sys.argv[4]
    if len(sys.argv) > 5:
        bagfile_range = sys.argv[5]
    if len(sys.argv) > 6:
        print("Too many arguments")
        return
    
    

    hist_timer = Timer(10)
    
    A = get_new_A(A)

    A['run_name'] = run_name
    A['loaded_collisions'] = None
    A['collisions'] = []
    collision_files = gg(opjD('collisions','*'))
    for c in collision_files:
        if run_name in c:
            A['loaded_collisions'] = lo(c)
            print 'loaded '+c
            break

    if bag_or_pkl == 'pkl':
        meta_path = opj(data_path,'meta',run_name)
        rgb_1to4_path = opj(data_path,'rgb_1to4',run_name)
        multi_preprocess_pkl_files(A,meta_path,rgb_1to4_path)

    elif bag_or_pkl == 'bag':
        #meta_path = '/media/karlzipser/ExtraDrive3/from_Mr_Yellow/Mr_Yellow_Fern_11April2017/processed/caffe2_z2_color_direct_local_11Apr17_22h14m05s_Mr_Yellow/.preprocessed2/left_image_bound_to_data.pkl'
        #path2 = '/media/karlzipser/ExtraDrive3/from_Mr_Yellow/Mr_Yellow_Fern_11April2017/processed/caffe2_z2_color_direct_local_11Apr17_22h14m05s_Mr_Yellow'
        meta_path = opj(data_path,run_name,'.preprocessed2/left_image_bound_to_data.pkl')
        bags_path = opj(data_path,run_name)
        multi_preprocess_bagfiles(A,meta_path,bags_path,bagfile_range=[])



    threading.Thread(target=animate.animate_with_key_control,args=[A]).start()
    #threading.Thread(target=animate.graph,args=[A]).start()
    #animate.graph(A)
    #raw_input()



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
    figure(1)
    clf()
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

    x_avgs = []
    y_avgs = []
    ctr = 0
    if True: #for i in range(300,2300,1):
        try:
            img = A['images'][int(A['current_img_index'])]
            #k = mci(img)
            #if k == ord('q'):
            #    break
            angles_to_center, angles_surfaces, distances_marker, markers = get_angles_and_distance(img) 
            #print angles_surfaces
            #print distances_marker
            print "---------------------"
            #fig=plt.figure(1)
            #plt.clf()
            #ax=fig.add_subplot(1,1,1)
            #xlim(-10,10)
            #ylim(-10,10)

            distance2 = 4*107/100.

            marker_ids = angles_to_center.keys()
            xlim(-5,5);ylim(-5,5)
            for m in marker_ids:
                if m in markers_xy_dic:
                    xy = markers_xy_dic[m]
                    angle1 = angles_to_center[m]
                    distance1 = distances_marker[m]
                    print(m,(np.degrees(marker_angles_dic[m])),np.degrees(angles_surfaces[m]),distances_marker[m])
                    angle2 = (np.pi+marker_angles_dic[m]) - (np.pi/2.0-angles_surfaces[m])
                    #angle2=angle1
                    #distance1 = 1
                    #if distance1 < 2:
                    #   plot_it2(angle1,distance1,angle2,distance2,xy)
                    xd = distance1 * np.sin(angle2)
                    yd = distance1 * np.cos(angle2)
                    xs = []
                    ys = []
                    if distance1 < 2*distance2 and distance1 > 0.05:
                        xs.append(xd+xy[0])
                        ys.append(yd+xy[1])
            x_avg = np.mean(array(xs))
            y_avg = np.mean(array(ys))
            x_avgs.append(x_avg)
            y_avgs.append(y_avg)
            #if np.mod(ctr,3) == 0:
            if len(x_avgs)>10:
                plot(array(x_avgs)[-10:].mean(),array(y_avgs[-10:]).mean(),'r.')
                pause(0.001)
            #ctr += 1
            #plot(array(x_avgs[-5:]).mean(),array(y_avgs[-5:]).mean(),'r.')
            #plot([xy[0],xd+xy[0]],[xy[1],yd+xy[1]])
            

                    #plot(x_avg,y_avg,'r.')
                    #pause(0.001)
            #raw_input('>')
        except:
           pass


        


if __name__ == '__main__':
    main()






if False:
    meta = sgg('/home/karlzipser/Desktop/bair_car_data_new/meta/*' )
    for i in range(len(meta)):

        meta[i] = fname(meta[i])
    meta = set(meta)

    ctr = 0
    collisions = sgg('/home/karlzipser/Desktop/collisions/*' )
    for i in range(len(collisions)):
        c = lo(collisions[i])
        ctr += c.sum()
        collisions[i] = fname(collisions[i]).split('.')[0]
    collisions = set(collisions)


