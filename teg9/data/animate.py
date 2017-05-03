from kzpy3.vis import *

def prepare_image(img,steer,motor,state,delay,scale,color_mode):
    
    bar_color = [0,0,0]
    if state == 1:
        bar_color = [0,0,255]
    elif state == 6:
        bar_color = [255,0,0]
    elif state == 5:
        bar_color = [255,255,0]
    elif state == 7:
        bar_color = [255,0,255]
    elif state == 2:
        bar_color = [100,100,100]
    else:
        bar_color = [0,0,0]
    if steer != None:
        apply_rect_to_img(img,steer,0,99,bar_color,bar_color,0.9,0.1,center=True,reverse=True,horizontal=True)
    if motor != None:
        apply_rect_to_img(img,motor,0,99,bar_color,bar_color,0.9,0.1,center=True,reverse=True,horizontal=False)
    
     
    if delay == None:
        scale_img = cv2.resize(img, (0,0), fx=scale, fy=scale)
        return scale_img
    else:
        k = mci(img,delay,'animate',scale)
        return k  

def animate(A):
    timer = Timer(1)
    ctr = 0
    while True:
        while A['STOP_ANIMATOR_THREAD'] == False:

            if len(A['images']) < 2*30:
                print 'waiting for images'
                time.sleep(1)
                continue
            if timer.check():
                print(d2s("A['current_img_index'] =", int(A['current_img_index'])))
                timer.reset()
            A['current_img_index'] += A['d_indx']
            if A['current_img_index'] >= len(A['images']):
                A['current_img_index'] = len(A['images'])-1
            elif A['current_img_index'] < 0:
                A['current_img_index'] = 0
            indx = int(A['current_img_index'])

            img = A['images'][indx].copy() #.copy() # Copy if need to change image.
            steer = A['steer'][indx]
            state = A['state'][indx]
            motor = A['motor'][indx]
            #print state

            if A['delay'] == None:
                print('Writing to file instead of display!')
                if len(A['images']) > A['save_stop_index']:
                    A['STOP_LOADER_THREAD'] = True
                if A['current_img_index'] > A['save_stop_index']:
                    A['STOP_ANIMATOR_THREAD'] = True
                    A['STOP_LOADER_THREAD'] = True
                    return
                if A['current_img_index'] >= A['save_start_index']:
                    out_img = prepare_image(img,steer,None,state,A['delay'],A['scale'],A['color_mode'])
                    imsave(opjD('temp2',d2n(ctr,'.png')),out_img)
                    print ctr
                    ctr += 1
                continue
            else:
                k = prepare_image(img,steer,None,state,A['delay'],A['scale'],A['color_mode'])

            if k == ord('q'):
                print('Exiting animate_thread.animate')
                A['STOP_LOADER_THREAD'] = True
                return
            if k == ord(' '):
                A['d_indx'] = 0
            if k == ord('1'):
                A['d_indx'] = 1
            if k == ord('2'):
                A['d_indx'] = 2
            if k == ord('3'):
                A['d_indx'] = 3
            if k == ord('4'):
                A['d_indx'] = 4
            if k == ord('5'):
                A['d_indx'] = 7
            if k == ord('6'):
                A['d_indx'] = 10
            if k == ord('7'):
                A['d_indx'] = 15
            if k == ord('8'):
                A['d_indx'] = 20
            if k == ord('9'):
                A['d_indx'] = 30   
            if k == ord('!'):
                A['d_indx'] = -1
            if k == ord('@'):
                A['d_indx'] = -2
            if k == ord('#'):
                A['d_indx'] = -3
            if k == ord('$'):
                A['d_indx'] = -4
            if k == ord('%'):
                A['d_indx'] = -7
            if k == ord('^'):
                A['d_indx'] = -10
            if k == ord('&'):
                A['d_indx'] = -15
            if k == ord('*'):
                A['d_indx'] = -20
            if k == ord('('):
                A['d_indx'] = -30
            if k == ord('w'):
                print("car ahead")
            if k == ord('a'):
                print("car left")
            if k == ord('d'):
                print("car right")

            if k == ord('z'):
                A['current_img_index'] = 0
            if k == ord('x'):
                A['current_img_index'] = 1*len(A['images'])/6
            if k == ord('c'):
                A['current_img_index'] = 2*len(A['images'])/6
            if k == ord('v'):
                A['current_img_index'] = 3*len(A['images'])/6
            if k == ord('b'):
                A['current_img_index'] = 4*len(A['images'])/6
            if k == ord('n'):
                A['current_img_index'] = 5*len(A['images'])/6
            if k == ord('m'):
                A['current_img_index'] = 6*len(A['images'])/6-1

            if k == ord('k'):
                A['current_img_index'] -= 2*30
            if k == ord('l'):
                A['current_img_index'] += 2*30


        time.sleep(0.2)







