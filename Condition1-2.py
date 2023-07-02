import pandas as pd
import time
import tkinter as tk
from tkinter import filedialog
import os
import json

root = tk.Tk()
root.withdraw()
folder_path = filedialog.askdirectory()
subfolders = os.listdir(folder_path)
print('\nProcessing...\n')
t1 = time.time()
#--------------------------------------------------------------------------------------------------------
t1 = -1.6
t2 = 180
A = 12.5
def condition1(df,t1,t2):
    try:
        df2 = df[(df['linear_acceleration.x_filtered'] < t1) & (df['msg.brake'] > t2)]
        # print(df)
        stamp = []
        loc1 = []
        t_list = list(df2['Time'])
        if t_list != []:
            stamp.append(t_list[0])
            for i in range(len(t_list)):
                if i == len(t_list)-1:
                    break
                if t_list[i+1]- t_list[i] > 5:
                    stamp.append(t_list[i+1])
            print('stamp1: ',stamp)
            loc1 = df.index[df['Time'].isin(stamp)].tolist()
            print('loc1: ', loc1)
            df.loc[loc1,'incident_flag'] = 1
        else:
            pass
    except KeyError as e:
        print(e)
        stamp = []
        loc1 = []
    return stamp, loc1
#--------------------------------------------------------------------------------------------------------
def condition2(df,subfolder,A):
    try:
        # map = pd.read_csv(os.path.join(folder_path, subfolder + r"/waypoints_map.csv"), index_col=0)
        subfolder = subfolder.replace('-','')
        subfolder = subfolder.replace('Chula','CU')
        vehicle,operator,location,date = subfolder.split('_')
        with open(f'D:\FOOH\Senior_Project\map\waypoint_{vehicle}_{location}.csv') as f:
            map = pd.read_csv(f)
        # map = map[map['event_flag'] != 0]
        x = list(map['x'])
        y = list(map['y'])

        df['in_station'] = 0
        shift_msg = df['msg.mode'].shift(periods=1)
        df2 = df[df['msg.mode'] != shift_msg]
        df2 = df2.iloc[1:]

        # A = 5 # bus stop area
        for i in range(len(x)):
            df2.loc[(abs((df2['pose.position.x']-x[i])) < A) & (abs((df2['pose.position.y']-y[i])) < A),'in_station'] = 1
        # print(df)
        stamp = df2[df2['in_station'] == 0]['Time']
        stamp = list(stamp)
        print('stamp2:', stamp)
        loc2 = df2.index[df2['in_station'] == 0].tolist()
        print('loc2: ', loc2)
        df.loc[loc2,'incident_flag'] = 1
        # remove mode switch at the start
        try:
            x0 = df['pose.position.x'].iat[0]
            y0 = df['pose.position.y'].iat[0]
            x1 = df['pose.position.x'].iat[loc2[0]]
            y1 = df['pose.position.y'].iat[loc2[0]]
            if abs(x1-x0) < A and abs(y1-y0) < A:
                stamp.pop(0)
                loc2.pop(0)
        except:
            pass
    except (FileNotFoundError, KeyError) as e:
        print(e)
        stamp = []
        loc2 = []
    return stamp, loc2
#--------------------------------------------------------------------------------------------------------
stamp_dict = {}
for subfolder in subfolders:
    try:
        df = pd.read_csv(os.path.join(folder_path, subfolder + r"/merged_filtered_df.csv"), index_col=0)
        print(subfolder)
        df['incident_flag'] = 0
        stamp1,loc1 = condition1(df,t1,t2)
        stamp2,loc2 = condition2(df,subfolder,A)
        stamp_dict[subfolder] = {}
        stamp_dict[subfolder]['Condition1'] = stamp1
        stamp_dict[subfolder]['Condition2'] = stamp2
        stamp_dict[subfolder]['loc1'] = loc1
        stamp_dict[subfolder]['loc2'] = loc2
        if 'in_station' in list(df.columns):
            df = df.drop(columns = 'in_station')
        df.to_csv(os.path.join(folder_path, subfolder + r"/merged_filtered_df.csv"))
        print('------ DONE ------\n')
    except FileNotFoundError as e:
        print(subfolder, e, '\n*Skip\n')
    
print(stamp_dict)
with open(f"Incidents_t1[{t1}]_t2[{t2}]_A[{A}].json", "w") as f: 
    jsonString = json.dumps(stamp_dict, indent=4)
    f.write(jsonString)
    
print('\n'+ '***'*10**2 + '\n')

