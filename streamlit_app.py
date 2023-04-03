import pandas as pd
import matplotlib.pyplot as plt
import pymongo
from pymongo import MongoClient
import streamlit as st
import numpy as np

st.set_page_config(page_title="Senior Project", page_icon="*", layout="wide")
st.title("Data of Autonomous Vehicles Testdrive")
st.markdown('#')

with open('style.css') as f:
    st.markdown(f'<style>{f.read()}</style>', unsafe_allow_html=True)

# RETRIEVE DATA FROM DB
connect = 'mongodb+srv://kopkap:kopkap123@cluster0.agjmc4n.mongodb.net/?retryWrites=true&w=majority' # Atlas
# connect = 'mongodb://localhost:27017' # Compass 
cluster = MongoClient(connect)
db1 = cluster["Testdrive"]
subfolders = db1.list_collection_names()

def loaddb(testdrive):
    collection = db1[testdrive]
    df = collection.find()
    df = pd.DataFrame(df, dtype = 'float64')
    df = df.drop(columns = ['_id','Unnamed: 0'])
    # x = df['msg.mode'][0]
    # print(x,type(x))
    return df
# st.write(subfolders)
#-----------------------------------------------------------------------------------------
def information():
    db2 = cluster["Information"]
    incident_col = db2['Incident']
    incident_dict = incident_col.find_one()
    info_col = db2['Testdrive_info']
    info = info_col.find()
    info = pd.DataFrame(info, dtype = 'float64')
    info = info.drop(columns = ['_id','Unnamed: 0'])
    st.header('Testdrives Information')
    st.write(info)
    return info,incident_dict
#-----------------------------------------------------------------------------------------
def readfile(testdrive, df):
    # with open(os.path.join(folder_path, testdrive +'/'+'waypoints_map.csv')) as f:
    #     map = pd.read_csv(f)
    testdrive = testdrive.replace('-','')
    testdrive = testdrive.replace('Chula','CU')
    vehicle,operator,location,date = testdrive.split('_')
    with open(f'waypoint_{vehicle}_{location}.csv') as f:
        map = pd.read_csv(f)

    # map = map[map['event_flag'] != 0]
    x = list(map['x'])
    y = list(map['y'])
    n = 5 # bus stop area
    temp = []
    for i in range(len(x)):
        temp.append(df[(abs((df['pose.position.x']-x[i])) < n) & (abs((df['pose.position.y']-y[i])) < n)]['Time'])
    df2 = pd.DataFrame(pd.concat(temp,axis = 0))
    return df2, map # df2 is list of bus stop
#-----------------------------------------------------------------------------------------
def boxes(testdrive,info):
    df = info
    index = df.loc[df['name'] == testdrive].index.values
    index = index[0]
    col1, col2, col3, col4 = st.columns(4)
    col1.metric('Location',str(df.iloc[index]['location']))
    col2.metric('Vehicle',str(df.iloc[index]['vehicle']))
    col3.metric('Date',str(df.iloc[index]['date']))
    col4.metric('Time', str(df.iloc[index]['time']))
#-----------------------------------------------------------------------------------------
def mileages_stat(info):
    q1,q2= st.columns(2,gap = 'large')
    with q1:
        df = info
        mileages_dict = {}
        gb = df.groupby(['vehicle'])
        for vehicle in ['T1','T2-B','T2-W']:
            mileages_dict[vehicle] = gb.get_group(vehicle)['mileages'].sum(axis=0)
        vehicles = list(mileages_dict.keys())
        mileages = list(mileages_dict.values())
        fig = plt.subplots(figsize = (5,4))
        barwidth = 0.4
        x1 = np.arange(len(mileages))
        x2 = [i + barwidth for i in x1]
        plt.bar(x1, mileages, color = 'orange',width = barwidth, label = 'Total Mileages')
        for i, v in enumerate(mileages):
            plt.text(float(i)-.2 , float(v)+100, str(round(v,2)), color='black', fontweight='light')
        #----------------------------------------------------
        auto_dict = {}
        for vehicle in ['T1','T2-B','T2-W']:
            auto = gb.get_group(vehicle)['mileages'] * gb.get_group(vehicle)['p_auto']/100
            auto_dict[vehicle] = auto.sum(axis=0)
        auto = list(auto_dict.values())
        plt.bar(x2, auto, color ='navy',width = barwidth, label = 'Autonomous')
        for i, v in enumerate(auto):
            plt.text(float(i)+.2 , float(v)+100, str(round(v,2)), color='black', fontweight='light')
        #----------------------------------------------------
        plt.legend(bbox_to_anchor=(1.0, 1), loc='upper left')
        plt.xticks([r + barwidth/2 for r in range(len(vehicles))],vehicles)
        plt.ylim(0,round(max(mileages),-3)+2000)
        plt.xlabel("Vehicle") 
        plt.ylabel("mileages(m)") 
        # plt.style.use('seaborn-whitegrid')
        st.header('Mileages Statistics')
        st.pyplot(plt) # streamlit
    #----------------------------------------------------
    with q2:
        df['month'] = df['date'].str[:-3]
        df['p'] = df['mileages'] * df['p_auto']/100
        # p = df['mileages'] * df['p_auto']
        gb = df.groupby('month')
        month_list = list(gb.groups.keys())
        auto_dict = {}
        mileages_dict = {}
        for m in month_list:
            a = gb.get_group(m)['p'].sum(axis=0)
            d = gb.get_group(m)['mileages'].sum(axis=0)
            ans = round(a/d*100,2)
            auto_dict[m] = ans
            mileages_dict[m] = d
            
        fig, ax1 = plt.subplots(figsize=(5,4))
        plt.style.use('seaborn-whitegrid')
        ax2 = ax1.twinx()
        
        x = auto_dict.keys()
        y = auto_dict.values()
        y2 = mileages_dict.values()

        ax1.set_ylim(0,round(max(y2),-3)+2000)
        ax2.set_ylim(0,100)
        ax1.bar(x, y2, color='orange',width = barwidth ,label = 'mileages')
        ax1.set_ylabel("Mileages") 

        ax2.plot(x, y, '-',color='blue',label = 'Auto%')
        ax1.set_xlabel("months") 
        ax2.set_ylabel("Autonomous(%)") 
        fig.legend(bbox_to_anchor=(1.0, 0.9), loc='upper left')
        # ax1.grid(False)
        ax2.grid(False)
        st.header('Autonomous Percentage Monthly')
        st.pyplot(plt)
    #----------------------------------------------------
    total_mileages = round(sum(mileages),2)
    total_auto = round(sum(auto),2)
    p_auto = round(total_auto/total_mileages*100,2)
    col1, col2, col3 = st.columns(3)
    col1.metric('Total Mileages(m)',str(total_mileages))
    col2.metric('Autonomous %',str(p_auto))
#-----------------------------------------------------------------------------------------
def graph1(testdrive, df, df2, incident_dict):
    st.header("Figure 1: Incident Occurances")
    plt.figure(figsize=(5,4))
    # plt.title('Plot 1')
    plt.xlabel("Time")
    plt.ylabel('Acceleration(m/s^2)')
    plt.style.use('seaborn-whitegrid')
    con1 = incident_dict[testdrive]['Condition1']
    con2 = incident_dict[testdrive]['Condition2']
    print(con1,con2)
    gb = df.groupby(['msg.mode'])
    t_auto = gb.get_group('True')['Time']
    t_manual = gb.get_group('False')['Time']
    h = 5
    # bottom layer
    plt.vlines(x=t_auto, ymin=-h, ymax=h, colors='cornflowerblue', ls='-', lw=1.5, label='Auto')
    plt.vlines(x=t_manual, ymin=-h, ymax=h, colors='lightcoral', ls='-', lw=1.5, label='Manual')
    plt.vlines(df2['Time'], ymin=-1, ymax=1, color = 'gold',label = 'Stops')
    plt.plot(df['Time'],df['linear_acceleration.x_filtered'],color = 'black',linestyle = '-',label = 'Acceleration_x')
    plt.vlines(x=con1, ymin=-h, ymax=h, colors='red', ls='-', lw=2, label='Condition1')
    plt.vlines(x=con2, ymin=-h, ymax=h, colors='blue', ls='--', lw=1.5, label='Condition2')
    # top layer
    plt.legend(bbox_to_anchor=(1.0, 1), loc='upper left')
    plt.style.use('seaborn-whitegrid')
    st.pyplot(plt) # streamlit
#-----------------------------------------------------------------------------------------
def graph2(testdrive, df, incident_dict, map):
    st.header("Figure 2: Incident locations")
    plt.figure(figsize=(5,4))
    # plt.title('Plot 2')
    plt.xlabel("X coordinates")
    plt.ylabel("Y coordinates")
    plt.scatter(df['pose.position.x'],df['pose.position.y'],color = 'lime',s = 3,label = 'Route')
    for index,row in map.iterrows():
        x = row['x']
        y = row['y']
        c=plt.Circle((x, y),5,color = 'gold')
        plt.gca().add_artist(c)
    loc1 = incident_dict[testdrive]['loc1']
    loc2 = incident_dict[testdrive]['loc2']
    plt.scatter(df.iloc[loc1]['pose.position.x'],df.iloc[loc1]['pose.position.y'],s=100,color = 'red',marker = '^',label = 'Condition1')
    plt.scatter(df.iloc[loc2]['pose.position.x'],df.iloc[loc2]['pose.position.y'],s=100,color = 'blue',marker = '^', label = 'Condition2')
    plt.legend(bbox_to_anchor=(1.0, 1), loc='upper left')
    plt.style.use('seaborn-whitegrid')
    st.pyplot(plt) # streamlit
#-----------------------------------------------------------------------------------------
def percent_mode(df):
    st.header("Percentage of driving modes")
    q1,q2= st.columns(2,gap = 'large')
    with q1:
        plt.figure()
        df['delta_x'] = df['pose.position.x'].diff()
        df['delta_y'] = df['pose.position.y'].diff()
        df['distance'] = (df['delta_x']**2 + df['delta_y']**2)**0.5
        auto = df[df['msg.mode'] == 'True']['distance'].sum(axis=0)
        d = df['distance'].sum(axis=0)
        auto = round(auto,2); d = round(d,2); manu = round(d-auto,2)
        p_auto = round(auto/d * 100,2)
        p_manu = round((100-p_auto),2)

        df2 = pd.DataFrame()
        df2['mode'] = ['auto','manual']
        df2['distance'] = [auto,manu]
        df2['Percentages'] = [p_auto, p_manu]

        # plt.title("Percentage of driving modes")
        plt.pie([auto,manu], labels=['Autonomous','Manual'], autopct='%1.1f%%',colors=['blue','red'])
        st.pyplot(plt) # streamlit
    with q2:
        st.write('Distance:',d)
        st.write('Auto:',auto,'Manual:',manu)
        st.write(df2)
#-----------------------------------------------------------------------------------------
def main():
    info, incident_dict = information()
    mileages_stat(info)
    st.markdown('#')
    st.header('Individual Testdrive Data')
    selected = st.selectbox('Select testdrive',tuple(subfolders))
    st.markdown('#')
    df = loaddb(selected)
    st.header('Viewing: '+str(selected))
    boxes(selected, info)
    df2, map = readfile(selected, df)
    testdrive = selected
    st.markdown('#')
    q1,q2= st.columns(2,gap = 'large')
    with q1:
        graph1(testdrive, df, df2, incident_dict)
    with q2:
        graph2(testdrive, df, incident_dict, map)
    st.markdown('#')
    st.write(df[df['incident_flag'] != 0])
    st.markdown('#')
    percent_mode(df)
#-----------------------------------------------------------------------------------------
main()
st.header('*'*70)
st.image('the-rock-eyebrow.gif')
