from kzpy3.vis import *
import math

N = lo(opjD('N.pkl'))
traj = N['Mr_Black']['direct_rewrite_test_28Apr17_17h23m15s_Mr_Black']['self_trajectory']

def rotatePoint(centerPoint,point,angle):
    """http://stackoverflow.com/questions/20023209/function-for-rotating-2d-objects
    Rotates a point around another centerPoint. Angle is in degrees.
    Rotation is counter-clockwise"""
    angle = math.radians(angle)
    temp_point = point[0]-centerPoint[0] , point[1]-centerPoint[1]
    temp_point = ( temp_point[0]*math.cos(angle)-temp_point[1]*math.sin(angle) , temp_point[0]*math.sin(angle)+temp_point[1]*math.cos(angle))
    temp_point = temp_point[0]+centerPoint[0] , temp_point[1]+centerPoint[1]
    return temp_point








def sample_gradient(xy,xy_prev,angles,pfield):

    sample_points = []
    potential_values = []

    delta_xy = xy - array(xy_prev)

    xy = array(xy)
    xy_forward = xy + delta_xy

    for angle in angles:
        pt = rotatePoint(xy,xy_forward,angle)
        sample_points.append(pt)

    for sp in sample_points:
        pix = meters_to_pixels(sp[0],sp[1])
        potential_values.append(pfield[pix[0],pix[1]])

    return sample_points,potential_values



from scipy.optimize import curve_fit

def f(x,A,B):
    return A*x+B


A,B = curve_fit(f,x,y)[0]

def xylim(a,b,c,d):
    xlim(a,b)
    ylim(c,d)
"""
def sqfig(a,b):
    figure(1,figsize=b)
    xylim(0,a,0,a)
"""

def normalized(a, axis=-1, order=2):
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2==0] = 1
    return a / np.expand_dims(l2, axis)


def normalized_vector_from_pts(pts):
    x = pts[:,0]
    y = pts[:,1]
    m,b = curve_fit(f,x,y)[0]
    heading = normalized([1,m])[0]
    return heading




Pts = zeros( (len(traj['left']['x']),2) )
Pts[:,0] = traj['left']['x']
Pts[:,1] = traj['left']['y']

pts = Pts.copy()

def get_sample_points(pts,angles,n=3):

    sample_points = []
    potential_values = []

    heading = normalized_vector_from_pts(pts[-n:,:])
    heading *= 0.5 # 50 cm, about the length of the car
    if pts[-n,0] > pts[-1,0]:
        heading *= -1
    #if pts[-3,1] > pts[-1,1]:
    #    heading *= -1

    for a in angles:

        sample_points.append( rotatePoint([0,0],heading,a) )
    #figure(3)
    #pts_plot(pts)
    for k in range(len(sample_points)):
        f = sample_points[k]
        #plot([pts[-1,0],pts[-1,0]+f[0]],[pts[-1,1],pts[-1,1]+f[1]])
    figure(1)
    for sp in sample_points:
        pix = meters_to_pixels(sp[0]+pts[-1,0],sp[1]+pts[-1,1])
        plot(pix[0],pix[1],'kx')
        potential_values.append(pfield[pix[0],pix[1]])

    return sample_points,potential_values




#pt_plot(pts[-1]+5*heading)

angles = range(-30,31,10)
angles = -1*array(angles)

mi(pfield,1)
xylim(-4,4,-4,4)
figure(2)
clf()
#xylim(0,7,1.5,1.57)
ctr = 0
for i in range(3000,len(pts),3):
    if traj['left']['t_vel'][i] > 0.4:
        pt = pts[i]
        pix = meters_to_pixels(pt[0],pt[1])
        sample_points,potential_values = get_sample_points(pts[i:i+6],angles,6)
        sp = sample_points[0]
        sp_pix = meters_to_pixels(sp[0]+pt[0],sp[1]+pt[1])
        if ctr ==0 or ctr > 10:
            figure(1)

            mi(pfield)
            
            
            figure(2)

            clf()
            #xylim(0,7,1.5,1.57)
            ctr = 0
        figure(1)
        plot(pix[0],pix[1],'r.')
        plot(sp_pix[0],sp_pix[1],'g.')
        figure(2)
        plot(potential_values)
        ctr += 1
        
        pause(0.1)
        #raw_input("dafds")#pause(0.03)

