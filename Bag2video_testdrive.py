# TEST DRIVE RECORD
import subprocess
from tkinter import image_names
import yaml
import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np
import glob
import os
from datetime import timedelta
import time
# import bagpy
# from bagpy import bagreader
########## USE PYTHON2 ################

st = time.time()

ROOT_DIR = r'/home/smrc/Desktop/ghan_senior_proj/'
path = r'/run/user/1000/gvfs/smb-share:server=smartmobi-nas.local,share=smartmobi/ROSBAG/'
file = r'T2-W_SL_CU_20230106165647'
bag = rosbag.Bag(path+file+'.bag')

TOPIC = '/camera0/image_raw'
print('Processing...\n')

path = ROOT_DIR + 'video/'
if not os.path.exists(path):
    os.mkdir(path)
    print('Directory created: ' + path)

path += file
if not os.path.exists(path):
    os.mkdir(path)
    print('Directory created: ' + path)

image_topic = bag.read_messages(TOPIC)

img_array = []
name_array = []
for k, b in enumerate(image_topic):
    # print(type(str(b.timestamp)))
    tstamp = b.timestamp
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(b.message, b.message.encoding)
    cv_image.astype(np.uint8) # # np.unit8 ???
#     cv2.imwrite(ROOT_DIR + 'NBTC_WP1_20221019193411/' + str(b.timestamp) + '.png', cv_image)
#     print('saved: ' + DESCRIPTION + str(b.timestamp) + '.png')

    img = cv_image
    height, width, layers = img.shape
    size = (width,height)
    img_array.append(img)
    name_array.append(str(tstamp))
#     name_array.append(filename[-23:-4])
    
bag.close()
print('Done Converting imgmsg to cv2...' + '\nimage no: ' + str(len(img_array)) )
print('\n'+'***'*10**2 +'\n')
# print(name_array)
#______________________________________________________________________________________________________________________
def genvid(Range):
    for i in Range:
        stamp = str((name_array[i]))
        td = timedelta(seconds=int(stamp[0:10])) # time lasp
        a = int(int(width)/2+120)
        b = int(int(height)/2+220)
        org = (a,b)
        temp = cv2.cvtColor(img_array[i], cv2.COLOR_RGB2BGR) # change color
        cv2.putText(temp, str(stamp)[:10]+'.'+str(stamp)[10:12], org, 2, color=(0, 0, 255), fontScale=0.5) #Add timestamp
        out.write(temp)
    out.release()

#______________________________________________________________________________________________________________________
img_array = img_array
name_array = name_array

fps = 10
vid_name = file + '.avi'

# path = ROOT_DIR + file
path += '/testdrive_rec/'
if not os.path.exists(path):
    os.mkdir(path)
    print('Directory created: '+ path)
    
out = cv2.VideoWriter(path + vid_name, cv2.VideoWriter_fourcc(*'DIVX'), fps, size)

genvid(range(0,len(img_array)))

vid_len = int(name_array[-1]) - int(name_array[0])
vid_len = round(vid_len/10**9,3)
td = timedelta(seconds=vid_len)

print('Done converting cv2 to video...' + '\nvideo name: ' + vid_name + '\tlength ' + str(td) + '\tfps: ' + str(len(img_array)/vid_len) + '\npath: ' + path)
et = time.time()
print('\nProcessing time: ' + str(et-st))
print('\n'+'***'*10**2 +'\n')