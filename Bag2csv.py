from re import S
import bagpy
from bagpy import bagreader
import pandas as pd
import os 

path = r'D:\FOOH\Senior_Project\bagfile\2020_10_21_Loop_2/' # path of folder
folders = os.listdir(path)
#-------------------------------------------------
def readfile(b):
    print(b.filename)
    print(b.start_time)
    print(b.topic_table)

    test_folder = path + str(b.filename)
    if not os.path.exists(test_folder):
        os.makedirs(test_folder)
        print('Directory created: ', test_folder)

    topics = ['/Vehicle_recv', '/vehicle_cmd', '/current_velocity', '/imu/data', '/current_pose', '/fix']
    # topics = ['/twist_cmd','/ublox/navsat','/ublox/navstatus','Vehicle_recv', 'vehicle_cmd', 'current_velocity', 'imu-data', 'current_pose', 'fix']
    for topic in topics:
        try:
            LASER_MSG = b.message_by_topic(topic)
            df = pd.read_csv(LASER_MSG)
            print(topic)
            # print(df)
            topic = topic.replace('/','')
            df.to_csv(os.path.join(test_folder, f'{topic}.csv'))
        except Exception as e:
            print(e)
#-------------------------------------------------
for file in folders:
    try:
        print(file)
        b = bagreader(path + file)
        readfile(b)
    except:
        pass

