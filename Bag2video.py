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

FILENAME = 'NBTC_WP1_20221019193411_SL.bag'
ROOT_DIR = 'D:/FOOH/Senior_Project/'
BAGFILE = ROOT_DIR +'bagfile/' + FILENAME 
bag = rosbag.Bag(BAGFILE)

TOPIC = '/image_raw'

image_topic = bag.read_messages(TOPIC)
# print(image_topic)
# print(enumerate(image_topic))

img_array = []
name_array = []
for k, b in enumerate(image_topic):
#     print(b.timestamp)
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
    name_array.append(int(tstamp))
#     name_array.append(filename[-23:-4])
    
bag.close()
print('Done Converting imgmsg to cv2...' + '\nimage no: ' + str(len(img_array)) )
print('-----------------------------------------')

#______________________________________________________________________________________________________________________
def genvid(Range):
    for i in range(Range):
        stamp = str(name_array[i])
        a = int(int(width)/2+120)
        b = int(int(height)/2+220)
        org = (a,b)
        cv2.putText(img_array[i], stamp, org, 2, color=(0, 0, 255), fontScale=0.5)
        out.write(img_array[i])

    out.release()

#______________________________________________________________________________________________________________________
img_array = img_array
name_array = name_array
# print(glob.glob('D:/FOOH/Senior_Project/images2video/*.jpg'))
# for filename in glob.glob('D:/FOOH/Senior_Project/images2video/*.jpg'):
# #     print(filename)
#     img = cv2.imread(filename) # np.unit8 ???
#     height, width, layers = img.shape
#     size = (width,height)
#     img_array.append(img)
#     name_array.append(filename)
# print(img_array)

fps = 10
vid_name = 'testdrive.avi'

path = ROOT_DIR + 'tdrive_rec/'
if not os.path.exists(path):
    os.makedir(path)
    print('Directory created: ', path)
    
out = cv2.VideoWriter(path + vid_name, cv2.VideoWriter_fourcc(*'DIVX'), fps, size)

genvid(range(0,len(img_array)))
# for i in range(len(img_array)):
#     stamp = str(name_array[i])
#     a = int(int(width)/2+120)
#     b = int(int(height)/2+220)
#     org = (a,b)
#     cv2.putText(img_array[i], stamp, org, 2, color=(0, 0, 255), fontScale=0.5)
#     out.write(img_array[i])
#     
# out.release()
vid_len = int(img_array[-1]) - int(img_array[0])
vid_len = round(vid_len/10**9,3)
td = timedelta(seconds=vid_len)
print('fps: ',fps = str(len(img_array)/vid_len))

print('Done converting cv2 to video...' + '\nvideo name: ' + vid_name + '\tlength' + str(td) + '\npath: ' + path)
print('-----------------------------------------')

#_____________________________________________________________________________________________________________________
# t = list of timestamps which incidents occur
t = ['1666182954259274286','1666182947435154674']
fps = 10

path = ROOT_DIR + 'incident_rec/'
if not os.path.exists(path):
    os.makedir(path)
    print('Directory created: ', path)
    
for j in range(len(t)):
    out = cv2.VideoWriter(path +'incident'+str(j+1)+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), fps, size)
    tStart = int(t[j]) - 10000000000
    tEnd = int(t[j]) + 5000000000
    # print(tStart,tEnd)
#     name_int = [int(i) for i in name_array]
    t1 = [str(i) for i in name_array if i >= tStart]
    t2 = [str(i) for i in name_array if i >= tEnd]
    t1 = t1[0]
    t2 = t2[0]
    t1 = name_array.index(t1)
    t2 = name_array.index(t2)
    
    genvid(range(t1,t2))
#     for i in range(t1,t2):
#         stamp = str(name_array[i])
#         a = int(int(width)/2+120)
#         b = int(int(height)/2+220)
#         org = (a,b)
#         cv2.putText(img_array[i], stamp, org, 2, color=(0, 0, 255), fontScale=0.5)
#         out.write(img_array[i])
#         
#     out.release()
print('Done collecting incident...' + '\nincident no: ' + str(len(t)) + '\npath: ' + path)
print('\n----------------Finished------------------')

#__________________________________________________________________________________________________________________

