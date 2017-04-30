from kzpy3.utils import *

import matplotlib
try:
    import cv2
except:
    print("Couldn't import cv")


MacOSX = False
if '/Users/' in home_path:
    MacOSX = True

if MacOSX:
    matplotlib.use(u'MacOSX')


###########
'''
e.g.
from kzpy3.vis import *; kzpy_vis_test()
'''
################

import matplotlib.pyplot as plt  # the Python plotting package
plt.ion()
plot = plt.plot
hist = plt.hist
xlim = plt.xlim
ylim = plt.ylim
clf = plt.clf
pause = plt.pause
figure = plt.figure
title = plt.title
plt.ion()
plt.show()
PP,FF = plt.rcParams,'figure.figsize'


def kzpy_vis_test():
    img_dic = get_some_images()
    ppff = PP[FF]
    PP[FF] = 3,3
    mi(img_dic['bay'],'bay')
    PP[FF] = ppff
    plt.figure('hist')
    plt.hist(np.random.randn(10000),bins=100)
    True

def hist(data,bins=100):
    """
    default hist behavior
    """
    plt.clf()
    plt.hist(data,bins=bins)
    pass
plot = plt.plot
figure = plt.figure
clf=plt.clf



try:
    # - These allow for real-time display updating
    from cStringIO import StringIO
    import scipy.ndimage as nd
    import PIL.Image
    if MacOSX:
        from IPython.display import clear_output, Image, display
    def showarray(a, fmt='jpeg'):
        a = np.uint8(np.clip(255.0*z2o(a), 0, 255))
        f = StringIO()
        PIL.Image.fromarray(a).save(f, fmt)
        display(Image(data=f.getvalue()))
except:
    print("kzpy3.vis: PIL image display not imported.")

def toolbar():
    plt.rcParams['toolbar'] = 'toolbar2'
    
######################
#
def mi(
    image_matrix,
    figure_num = 1,
    subplot_array = [1,1,1],
    img_title = '',
    img_xlabel = 'x',
    img_ylabel = 'y',
    cmap = 'gray',
    toolBar = True,
    do_clf = True,
    do_axis = False ):
    """
    My Imagesc, displays a matrix as grayscale image if 2d, or color if 3d.
    Can take different inputs -- e.g.,

        from matrix:

            from kzpy3.vis import *
            mi(np.random.rand(256,256),99,[1,1,1],'random matrix')

        from path:
            mi(opjh('Desktop','conv1'),1,[5,5,0])

        from list:
            l = load_img_folder_to_list(opjh('Desktop','conv5'))
            mi(l,2,[4,3,0])

        from dict:
            mi(load_img_folder_to_dict(opjh('Desktop','conv5')),1,[3,4,0])
    """
    if type(image_matrix) == str:
        mi(load_img_folder_to_dict(image_matrix),image_matrix,subplot_array,img_title,img_xlabel,img_ylabel,cmap,toolBar)
        return

    if type(image_matrix) == list:
        if np.array(subplot_array).max() < 2:
            subplot_array = [1,len(image_matrix),0]
        for i in range(len(image_matrix)):
            mi(image_matrix[i],figure_num,[subplot_array[0],subplot_array[1],i+1],img_title,img_xlabel,img_ylabel,cmap,toolBar)
        return

    if type(image_matrix) == dict:
        if np.array(subplot_array).max() < 2:
            subplot_array = [1,len(image_matrix),0]
        i = 0
        img_keys = sorted(image_matrix.keys(),key=natural_keys)
        for k in img_keys:
            mi(image_matrix[k],figure_num,[subplot_array[0],subplot_array[1],i+1],img_title,img_xlabel,img_ylabel,cmap,toolBar)
            i += 1
        return        

    if toolBar == False:
        plt.rcParams['toolbar'] = 'None'
    else:
        plt.rcParams['toolbar'] = 'toolbar2'

    f = plt.figure(figure_num)
    if do_clf:
        #print('plt.clf()')
        plt.clf()

    if True:
        f.subplots_adjust(bottom=0.05)
        f.subplots_adjust(top=0.95)
        f.subplots_adjust(wspace=0.1)
        f.subplots_adjust(hspace=0.1)
        f.subplots_adjust(left=0.05)
        f.subplots_adjust(right=0.95)
    if False:
        f.subplots_adjust(bottom=0.0)
        f.subplots_adjust(top=0.95)
        f.subplots_adjust(wspace=0.0)
        f.subplots_adjust(hspace=0.1)
        f.subplots_adjust(left=0.0)
        f.subplots_adjust(right=1.0)
    f.add_subplot(subplot_array[0],subplot_array[1],subplot_array[2])
    imgplot = plt.imshow(image_matrix, cmap)
    imgplot.set_interpolation('nearest')
    if not do_axis:
        plt.axis('off')
    if len(img_title) > 0:# != 'no title':
        plt.title(img_title)
#
######################









def mp(args,figure_num=1, subplot_array=[1,1,1],
       title='', xlabel='', ylabel='', xlim=[], ylim=[], toolBar=False):

    if toolBar == False:
        plt.rcParams['toolbar'] = 'None'
    else:
        plt.rcParams['toolbar'] = 'toolbar2'

    f = plt.figure(figure_num)

    if False:
        f.subplots_adjust(bottom=0.05)
        f.subplots_adjust(top=0.95)
        f.subplots_adjust(wspace=0.1)
        f.subplots_adjust(hspace=0.1)
        f.subplots_adjust(left=0.05)
        f.subplots_adjust(right=0.95)

    f.add_subplot(subplot_array[0],subplot_array[1],subplot_array[2])
    imgplot = plt.plot(*args)
    if len(title) > 0:# != 'no title':
        plt.title(title)
    else:
        plt.title(str(subplot_array[2]))
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    if len(xlim)==2:
        plt.xlim(xlim)    
    if len(ylim)==2:
        plt.ylim(ylim)


def yb_color_modulation_of_grayscale_image(img,y,b,opt_lower_contrast=True):

    if len(np.shape(img))>2:
        img = np.mean(img,axis=2)
    img = z2o(img)

    if opt_lower_contrast:
        print('low contrast option')
        img = (1.0+img)/3.0

    y = z2o(y)
    b = z2o(b)

    ci = np.zeros((np.shape(img)[0],np.shape(img)[1],3))
    print(np.shape(ci))
    for i in range(3):
        ci[:,:,i] = 1.0*img
    ci = ci/np.max(ci)

    for i in range(3):
        ci[:,:,i] *= (1-y)
    for i in [0,1]:
        ci[:,:,i] += y

    for i in range(3):
        ci[:,:,i] *= (1-b)
    for i in [2]:
        ci[:,:,i] += b
        
    return ci

    
 
def get_some_images():
    '''
    Load some images that can be used for demos, etc.
    e.g., img_dic = get_some_images(); mi(img_dic['bay'])
    '''
    img_dic = {}
    img_dic['bay'] = imread(opj(home_path,'Pictures','bay2.png'))
    return img_dic



# take an array of shape (n, height, width) or (n, height, width, channels)
# and visualize each (height, width) thing in a grid of size approx. sqrt(n) by sqrt(n)
def vis_square(data_in, padsize=1, padval=0):
    data = data_in.copy()
    data -= data.min()
    data /= data.max()
    
    # force the number of filters to be square
    n = int(np.ceil(np.sqrt(data.shape[0])))
    padding = ((0, n ** 2 - data.shape[0]), (0, padsize), (0, padsize)) + ((0, 0),) * (data.ndim - 3)
    data = np.pad(data, padding, mode='constant', constant_values=(padval, padval))
    
    # tile the filters into an image
    data = data.reshape((n, n) + data.shape[1:]).transpose((0, 2, 1, 3) + tuple(range(4, data.ndim + 1)))
    data = data.reshape((n * data.shape[1], n * data.shape[3]) + data.shape[4:])
    
    return data



import matplotlib.colors
def make_colormap(seq):
    """Return a LinearSegmentedColormap
    seq: a sequence of floats and RGB-tuples. The floats should be increasing
    and in the interval (0,1).
    """
    seq = [(None,) * 3, 0.0] + list(seq) + [1.0, (None,) * 3]
    cdict = {'red': [], 'green': [], 'blue': []}
    for i, item in enumerate(seq):
        if isinstance(item, float):
            r1, g1, b1 = seq[i - 1]
            r2, g2, b2 = seq[i + 1]
            cdict['red'].append([item, r1, r2])
            cdict['green'].append([item, g1, g2])
            cdict['blue'].append([item, b1, b2])
    return matplotlib.colors.LinearSegmentedColormap('CustomMap', cdict)

''' from http://stackoverflow.com/questions/16834861/create-own-colormap-using-matplotlib-and-plot-color-scale
e.g.,

c = matplotlib.colors.ColorConverter().to_rgb
rvb = make_colormap(
    [c('red'), c('violet'), 0.33, c('violet'), c('blue'), 0.66, c('blue')])
N = 1000
array_dg = np.random.uniform(0, 10, size=(N, 2))
colors = np.random.uniform(-2, 2, size=(N,))
plt.scatter(array_dg[:, 0], array_dg[:, 1], c=colors, cmap=rvb)
plt.colorbar()
plt.show()
'''





def load_img_folder_to_dict(img_folder):
    '''Assume that *.* selects only images.'''
    img_fns = gg(opj(img_folder,'*.*'))
    imgs = {}
    for f in img_fns:
        imgs[f.split('/')[-1]] = imread(f)
    return imgs

def load_img_folder_to_list(img_folder):
    return dict_to_sorted_list(load_img_folder_to_dict(img_folder))



def my_scatter(x,y,xmin,xmax,fig_wid,fig_name):
    plt.figure(fig_name,(fig_wid,fig_wid))
    plt.clf()
    plt.plot(x,y,'bo')
    plt.title(np.corrcoef(x,y)[0,1])
    plt.xlim(xmin,xmax)
    plt.ylim(xmin,xmax)



def apply_rect_to_img(img,value,min_val,max_val,pos_color,neg_color,rel_bar_height,rel_bar_thickness,center=False,reverse=False,horizontal=False):
    #print(value)
    h,w,d = shape(img)
    p = (value - min_val) / (max_val - 1.0*min_val)
    if reverse:
        p = 1.0 - p
    if p > 1:
        p = 1
    if p < 0:
        p = 0
    wp = int(p*w)
    hp = int(p*h)
    bh = int((1-rel_bar_height) * h)
    bt = int(rel_bar_thickness * h)
    bw = int((1-rel_bar_height) * w)

    if horizontal:
        if center:
            if wp < w/2:
                img[(bh-bt/2):(bh+bt/2),(wp):(w/2),:] = neg_color
            else:
                img[(bh-bt/2):(bh+bt/2),(w/2):(wp),:] = pos_color
        else:
            img[(bh-bt/2):(bh+bt/2),0:wp,:] = pos_color
    else:
        if center:
            if hp < h/2:
                img[(hp):(h/2),(bw-bt/2):(bw+bt/2),:] = neg_color
            else:
                img[(h/2):(hp),(bw-bt/2):(bw+bt/2),:] = pos_color

        else:
            img[hp:h,(bw-bt/2):(bw+bt/2),:] = pos_color


def plt_square():
    plt.gca().set_aspect('equal',adjustable='box')
    plt.draw()



def function_close_all_windows():
    plt.close('all')
CA = function_close_all_windows



def mi_or_cv2_animate(img_array,cv=True,delay=30,title='animate'):
    if type(img_array)==np.ndarray:
        for i in range(len(img_array)):
             mi_or_cv2(img_array[i],cv,delay,title)        
    elif type(img_array)==np.ndarray:
        for i in range(len(img_array[0])):
             mi_or_cv2(img_array[i],cv,delay,title)
    else:
        print('I am confused')
        assert(False)

def mi_or_cv2(img,cv=True,delay=30,title='animate'):
    if cv:
        cv2.imshow(title,cv2.cvtColor(img,cv2.COLOR_RGB2BGR))
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            pass
    else:
        mi(img,title)
        pause(0.0001)

