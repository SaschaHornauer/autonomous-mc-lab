



def makeGaussian(size, fwhm = 3, center=None):
    """ Make a square gaussian kernel.

    size is the length of a side of the square
    fwhm is full-width-half-maximum, which
    can be thought of as an effective radius.
    http://stackoverflow.com/questions/7687679/how-to-generate-2d-gaussian-with-python

    """

    x = np.arange(0, size, 1, float)
    y = x[:,np.newaxis]

    if center is None:
        x0 = y0 = size // 2
    else:
        x0 = center[0]
        y0 = center[1]

    return np.exp(-4*np.log(2) * ((x-x0)**2 + (y-y0)**2) / fwhm**2)




def iadd(src,dst,xy,neg=False):
    src_size = []
    upper_corner = []
    lower_corner = []
    for i in [0,1]:
        src_size.append(shape(src)[i])
        upper_corner.append(int(xy[i]-src_size[i]/2.0))
        lower_corner.append(int(xy[i]+src_size[i]/2.0))
    if neg:
        dst[upper_corner[0]:lower_corner[0],upper_corner[1]:lower_corner[1]] -= src
    else:
        dst[upper_corner[0]:lower_corner[0],upper_corner[1]:lower_corner[1]] += src
    
    


from kzpy3.teg9.data.markers_clockwise import markers_clockwise
Origin = 300
Mult = 50
pfield = zeros((2*Origin,2*Origin))
marker_ids_all = []
marker_angles_dic = {}
marker_angles = 2*np.pi*np.arange(len(markers_clockwise))/(1.0*len(markers_clockwise))
marker_xys = []
for i in range(len(markers_clockwise)):
    a = marker_angles[i]
    marker_angles_dic[markers_clockwise[i]] = a
    x = 4*107/100.*np.sin(a)
    y = 4*107/100.*np.cos(a)
    marker_xys.append([x,y])
markers_xy_dic = {}
assert(len(markers_clockwise) == len(marker_xys))

def meters_to_pixels(x,y):
    return (int(-Mult*x)+Origin),(int(Mult*y)+Origin)

pfield *= 0
gm = makeGaussian(150,33*1.5)
gc = makeGaussian(300,80)
gs = makeGaussian(40,12)
for j in range(len(markers_clockwise)):
    m = markers_clockwise[j]
    xy = marker_xys[j]
    markers_xy_dic[m] = xy
    xp,yp = meters_to_pixels(xy[0],xy[1])
    print((xp,yp))
    iadd(gm,pfield,(xp,yp))
    iadd(5*gs,pfield,(xp,yp))
iadd(2*gc,pfield,(Origin,Origin))
mi(pfield,'pfield')



def sample_gradient(xy,xy_prev,angles,pfield,PLOT=True):
    """
    xy = [3,2.9]
    xy_prev = [3-0.05,2.9]
    dt = 0
    alpha = 5
    n_samples = 5
    """
    xy = array(xy)
    xy_prev = array(xy_prev)
    xy_forward = 2*xy - xy_prev
    sample_points = []
    potential_values = []
    for angle in angles:
        pt = rotatePoint(xy,xy_forward,angle)
        sample_points.append(pt)

    if PLOT:
        plot(xy[0],xy[1],'r.');pause(0.000001)
        #plot([xy[0],xy_prev[0]],[xy[1],xy_prev[1]],'r')
    for sp in sample_points:
        pix = meters_to_pixels(sp[0],sp[1])
        potential_values.append(pfield[pix[0],pix[1]])
        if False:#PLOT:
            plot([xy[0],sp[0]],[xy[1],sp[1]],'g')
            pause(0.000001)

    return sample_points,potential_values

figure('sg',figsize=(5,5))
clf()
xlim(-4,4)
ylim(-4,4)
xy = [-2,0]
xy_prev = [xy[0],xy[1]-0.05]
dt = 0
alpha = 5
n_samples = 5

for j in range(20):
    print j
    for i in range(200):
        if mod(i,200000000) == 0:
            PLOT = True
        #if mod(i,10) == 0:
         #   
        else:
            PLOT = False
        sample_points,potential_values = sample_gradient(xy,xy_prev,[-2,-1,0,1,2],pfield,PLOT)
        
        xy_prev = xy
        
        if np.random.random()>0.1:
            c = potential_values.index(min(potential_values))
        else:
            c = random.choice([0,1,2,3,4])
        xy = sample_points[c]
       

