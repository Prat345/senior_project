import pandas as pd
import tkinter as tk
from tkinter import filedialog
import os
import sys
import time
from scipy.signal import butter,filtfilt
import numpy as np
import time
from datetime import timedelta
import warnings

# RUN IN TERMINAL
topics = {}
topic_names = ['Vehicle_recv', 'vehicle_cmd', 'current_velocity', 'imu-data', 'current_pose', 'fix']
root = tk.Tk()
root.withdraw()
# SELECT FOLDER THAT HAS ALL CSV FILES
folder_path = filedialog.askdirectory()
# SUBFOLER = EACH TESTDRIVE FOLDER 
subfolders = os.listdir(folder_path)
# print(subfolders)
t1 = time.time()
#-------------------------------------------------------------------------------------------
# Filter requirements.
T = 10         # Sample Period
fs = 1000     # sample rate, Hz
cutoff = 2     # desired cutoff frequency of the filter, Hz ,      slightly higher than actual 1.2 Hz
nyq = 0.5 * fs  # Nyquist Frequency
order = 2      # sin wave can be approx represented as quadratic
n = int(T * fs) # total number of samples

def butter_lowpass_filter(data, cutoff, fs, order):
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

def lowpass_filter(df,var):
     data = df[var].to_numpy()
     y = butter_lowpass_filter(data, cutoff, fs, order)
     return y
#----------------------------------------------------------------------------------------
# 1. LOW PASS FILTER + TIME SYNC
def step1(subfolder):
    for topic_name in topic_names:
        try:
            df = pd.read_csv(os.path.join(folder_path, subfolder, topic_name + ".csv"))
            if topic_name == 'imu-data': # remove noise from acceleration
                try:
                    df['linear_acceleration.x_filtered'] = lowpass_filter(df,'linear_acceleration.x')
                except KeyError as e:
                    print(subfolder, e)
        except FileNotFoundError as e:
            print(topic_name, e)
        try:
            df["date_time"] = pd.to_datetime((df["Time"] + 25200) * 1000000000.0)
        except KeyError as e:
            print(topic_name, e)
        topics[topic_name] = df

    merged_df = pd.DataFrame()
    # print(topics)
    for topic in topics:
        if merged_df.empty:
            merged_df = topics[topic]
        else:
            merged_df = pd.merge_ordered(merged_df, topics[topic], on="Time", fill_method="ffill")
            warnings.simplefilter(action='ignore', category=FutureWarning) # SURPRESS WARNING
    merged_df.dropna(how='all', axis=1, inplace=True)
    merged_df.dropna(how='any', axis=0, inplace=True)
    # print(merged_df)
    return merged_df

#--------------------------------------------------------------------------------------------------
# 2. FILTER TOPICS
def step2(merged_df):
    keep = ['Time','msg.mode','msg.brake','msg.steering_angle','msg.steering_angle_deg',
        'twist.linear.x','twist.linear.y','twist.linear.z',
        'linear_acceleration.x','linear_acceleration.x_filtered','linear_acceleration.y','linear_acceleration.z',
        'pose.position.x','pose.position.y','pose.position.z',
        ]
    merged_df = merged_df[keep]
    return merged_df
#-------------------------------------------------------------------------------------------------
# 3. ADD ELASPE TIME
def step3(df):
    position = df.columns.get_loc('Time')
    df['elasped_sec'] = (df.iloc[1:, position] - df.iat[0, position])
    df['elasped_sec'].fillna(0.000,inplace = True)

    df['elasped_time'] = pd.to_datetime(df['Time'],unit = 's')
    # df['elasped_time'] = df['elasped_time'].dt.strftime('%H:%M:%S')
    position = df.columns.get_loc('elasped_time')
    df['elasped_time'] =  (df.iloc[1:, position] - df.iat[0, position])
    df['elasped_time'].fillna(timedelta(seconds = 0.0),inplace = True)
    df['elasped_time'] = df['elasped_time'].dt.round('s')
    # print(df)
    df['elasped_time'] = df['elasped_time'].astype(str).str[7:]
    # print(df)
    return df
#-------------------------------------------------------------------------------------------------
# 3. CALCULATE MILEAGES
def step4(df):
    df['mileages'] = 0
    try:
        delta_x = df['pose.position.x'].diff()
        delta_y = df['pose.position.y'].diff()
        distance = (delta_x**2 + delta_y**2)**0.5
        df['mileages'] = distance.cumsum()
        df.iat[0, df.columns.get_loc('mileages')] = 0
    except KeyError as e:
        print('No topic', e)
    return df
#-------------------------------------------------------------------------------------------------
# 4. SAVE TO CSV
def savefile(df, subfolder):
    merged_df = df.reset_index() # Reset index
    merged_df =merged_df.drop(columns = 'index')

    # new_path = os.path.join('D:\FOOH\Senior_Project\Testdrives2', subfolder)
    # if not os.path.exists(new_path):
    #     os.makedirs(new_path)
    #     print('Directory created: ', new_path)
    # folder_path = new_path
    merged_df.to_csv(os.path.join(folder_path, subfolder + "/merged_filtered_df.csv"))
    print('--------- DONE ---------\n' )
#-------------------------------------------------------------------------------------------------
# MAIN
def process_csv():
    for subfolder in subfolders:
        print(f'\n[Processing] {subfolder}')
        temp = os.listdir(os.path.join(folder_path,subfolder))
        temp = [i.split('.')[0] for i in temp]
        temp = [i for i in topic_names if i in temp]
        # print(temp)
        if temp != []:
            merged_df = step1(subfolder)
            df = step2(merged_df)
            df = step3(df)
            df = step4(df)
            savefile(df,subfolder)
        else:
            print('[Skip] ' + subfolder)

process_csv()
t3 = time.time()
print(t3-t1)
print('\n'+ '***'*10**2 + '\n')

