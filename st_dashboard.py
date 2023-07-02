import pandas as pd
import matplotlib.pyplot as plt
import pygal
import pymongo
from pymongo import MongoClient
import streamlit as st
import numpy as np
import base64
from datetime import datetime
import matplotlib.dates as mdates

# st.cache(hash_funcs={"MyUnhashableClass": lambda _: None})
st.set_page_config(page_title="Senior Project", page_icon="*", layout="wide")
st.title("Test drive Dashboard")
st.markdown('#')

color1 = 'teal'
color2 = 'darkorange'
color3 = 'royalblue' # auto
color4 = 'limegreen' # manual
color5 = 'violet'
with open('style2.css') as f:
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
    df.drop(columns = ['_id','Unnamed: 0'], inplace=True)
    position = df.columns.get_loc('Time')
    starttime = df.iat[0, position]
    df['elasped_sec'] = (df.iloc[1:, position] - starttime)
    df['elasped_sec'].fillna(0.000,inplace = True)
    df['elasped_time'] = pd.to_datetime(df['elasped_sec'],unit = 's')
    return df, starttime
# #-----------------------------------------------------------------------------------------
def information():
    db2 = cluster["Information"]
    incident_col = db2['Incident']
    incident_dict = incident_col.find_one()
    info_col = db2['Testdrive_info']
    info = info_col.find()
    info = pd.DataFrame(info, dtype = 'float64') 
    info = info.drop(columns = ['_id','Unnamed: 0'])
    # st.header('Testdrives Information')
    # st.write(info)
    return info,incident_dict
#-----------------------------------------------------------------------------------------
def readfile(testdrive, df):
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
        temp.append(df[(abs((df['pose.position.x']-x[i])) < n) & (abs((df['pose.position.y']-y[i])) < n)]['elasped_time'])
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

    date = df.iloc[index]['date']
    date = datetime.strptime(date, '%Y-%m-%d').strftime('%d/%m/%Y')
    col3.metric('Date',date)
    col4.metric('Time', str(df.iloc[index]['time']))
#-----------------------------------------------------------------------------------------
def mileages_stat(info,n):
    df = info
    mileages_dict = {}
    gb = df.groupby(['vehicle'])
    auto_dict = {}
    for vehicle in ['T1','T2-B','T2-W']:
        mileages_dict[vehicle] = gb.get_group(vehicle)['mileages'].sum(axis=0)
        temp = gb.get_group(vehicle)
        auto = (temp['p_auto'] * temp['mileages']).sum(axis=0)/temp['mileages'].sum(axis=0)
        auto_dict[vehicle] = auto.sum(axis=0)
    vehicles = list(mileages_dict.keys())
    mileages_list = list(mileages_dict.values())
    auto_list = list(auto_dict.values())
    manual_list = [100-float(i) for i in auto_list]
    #-------------------------------------------------------
    total_mileages = round(df['mileages'].sum(axis=0),2)
    df['auto_mileages'] = df['mileages'] * df['p_auto']/100
    p_auto = df['auto_mileages'].sum(axis=0)/total_mileages * 100
    p_auto = round(p_auto,2)
    col1, col2, col3 = st.columns(3)
    col1.metric('Total Mileages',str(total_mileages) + ' m', delta=+2512.07, delta_color="normal")
    col2.metric('Average Autonomous %',str(p_auto)+ '%', delta=-3.71, delta_color="normal")
    col3.metric('Total Test drives',str(n))
    #-------------------------------------------------------
    q1,q2= st.columns(2,gap = 'large')
    with q1:
        fig, ax1 = plt.subplots(figsize=(5,4), dpi=200)
        ax2 = ax1.twinx()
        ax1.set_ylim(0,round(max(mileages_list),-3)+4000)
        ax2.set_ylim(0,100)
        barwidth = 0.4
        ax1.plot(vehicles, mileages_list, color = color2, lw = 3, marker = '^',markeredgecolor = 'black',label = 'Total Mileages')
        for i, v in enumerate(mileages_list):
            ax1.text(float(i)-.3 , float(v)+300, str(round(v))+' m', color='black', fontweight='bold')
        #----------------------------------------------------
        ax2.bar(vehicles, auto_list, width = barwidth, color=color3, label = 'Auto%')
        ax2.bar(vehicles, manual_list, bottom = auto_list, width = barwidth, color=color4, label = 'Manual%')
        for i, v in enumerate(auto_list):
            ax2.text(float(i)-.15 , float(v)/2, str(round(v,2))+'%', color='white', fontweight='light')
            ax2.text(float(i)-.15 , float(manual_list[i])/2+float(v), str(round(manual_list[i],2))+ '%', color='white', fontweight='light')
        #----------------------------------------------------
        ax1.set_ylabel("Mileages(m)") 
        ax1.set_xlabel("Vehicles") 
        ax2.set_ylabel("Autonomous(%)") 
        fig.legend(bbox_to_anchor=(0.5, 0.9), loc='lower center', ncol = 3, fancybox = True)
        ax1.grid(False)
        ax2.grid(True)
        st.markdown('Mileages by vehicle')
        ax1.set_zorder(ax2.get_zorder()+1)
        ax1.patch.set_visible(False)
        st.pyplot(plt) # streamlit
    #----------------------------------------------------
    with q2:
        df['month'] = df['date'].str[:-3]
        # p = df['mileages'] * df['p_auto']
        gb = df.groupby('month')
        month_list = list(gb.groups.keys())
        auto_dict = {}
        mileages_dict = {}
        for m in month_list:
            a = gb.get_group(m)['auto_mileages'].sum(axis=0)
            d = gb.get_group(m)['mileages'].sum(axis=0)
            ans = round(a/d*100,2)
            auto_dict[m] = ans
            mileages_dict[m] = d
            
        fig, ax1 = plt.subplots(figsize=(5,4), dpi=200)
        ax2 = ax1.twinx()
        
        month_list = list(auto_dict.keys())
        auto_list = list(auto_dict.values())
        manual_list = [100-float(i) for i in auto_list]
        mileages_list = list(mileages_dict.values())

        ax1.set_ylim(0,round(max(mileages_list),-3)+4000)
        ax2.set_ylim(0,100)

        ax2.bar(month_list, auto_list, width = barwidth, color=color3, label = 'Auto%')
        ax2.bar(month_list, manual_list, bottom = auto_list, width = barwidth, color=color4, label = 'Manual%')
        ax1.plot(month_list, mileages_list, color=color2,lw = 3,marker = '^',markeredgecolor = 'black',label = 'Total Mileages')
        for i, v in enumerate(mileages_list):
            ax1.text(float(i)-.3 , float(v)+300, str(round(v))+' m', color='black', fontweight='bold')
        for i, v in enumerate(auto_list):
            ax2.text(float(i)-.15 , float(v)/2, str(round(v,2))+'%', color='white', fontweight='light')
            ax2.text(float(i)-.15 , float(manual_list[i])/2+float(v), str(round(manual_list[i],2))+ '%', color='white', fontweight='light')

        ax1.set_ylabel("Mileages(m)") 
        ax1.set_xlabel("Months") 
        ax2.set_ylabel("Autonomous(%)") 
        fig.legend(bbox_to_anchor=(0.5, 0.9), loc='lower center', ncol = 3, fancybox = True)
        ax1.grid(False)
        ax2.grid(True)
        st.markdown('Mileages by Month')
        ax1.set_zorder(ax2.get_zorder()+1)
        ax1.patch.set_visible(False)
        st.pyplot(plt)
#-----------------------------------------------------------------------------------------
def plotg(df,title, topic, y_label):
    st.markdown(f"{title}")
    fig, ax = plt.subplots(figsize=(8,6))
    ax.set_xlabel("Elaspe Time")  
    ax.set_ylabel(y_label)
    ax.grid(True)
    ax.plot(df['elasped_time'],df[topic],color = color1, linestyle = '-', lw=2, label = topic)
    myFmt = mdates.DateFormatter('%M:%S')
    ax.xaxis.set_major_formatter(myFmt)
    fig.autofmt_xdate()
    st.pyplot(fig) # streamlit
#-----------------------------------------------------------------------------------------
def graph1(testdrive, df, df2, incident_dict, starttime):
    st.markdown("Incident Timeframe")
    fig, ax = plt.subplots(figsize=(8,6), dpi=200)
    # plt.title('Plot 1')
    ax.set_xlabel("Elaspe Time")
    ax.set_ylabel('Acceleration(m/s^2)')
    ax.grid(True)
    con1 = pd.Series(incident_dict[testdrive]['Condition1']) - starttime
    con2 = pd.Series(incident_dict[testdrive]['Condition2']) - starttime
    con1 = pd.to_datetime(con1,unit = 's')
    con2 = pd.to_datetime(con2,unit = 's')
    gb = df.groupby(['msg.mode'])
    t_auto = gb.get_group('True')['elasped_time']
    t_manual = gb.get_group('False')['elasped_time']
    h = 3
    # bottom layer
    ax.vlines(x=t_auto, ymin=-h, ymax=h, colors= 'lightskyblue', ls='-', lw=3, label='Auto',alpha = 0.2)
    ax.vlines(x=t_manual, ymin=-h, ymax=h, colors= 'palegreen', ls='-', lw=3, label='Manual',alpha = 0.2)
    ax.vlines(df2['elasped_time'], ymin=-1, ymax=1, color = color5,label = 'Station',alpha = 0.2)
    ax.plot(df['elasped_time'],df['linear_acceleration.x_filtered'],color = color1,linestyle = '-',label = 'Acc_x',lw = 1)
    ax.vlines(x=con1, ymin=-h, ymax=h, colors='red', ls='-', lw=3, label='Condition1')
    ax.vlines(x=con2, ymin=-h, ymax=h, colors='blue', ls='--', lw=2, label='Condition2')
    # top layer
    ax.legend(bbox_to_anchor=(0.5, 1.05), loc='lower center', ncol = 3, fancybox = True, labelspacing = 1)
    myFmt = mdates.DateFormatter('%M:%S')
    ax.xaxis.set_major_formatter(myFmt)
    fig.autofmt_xdate()
    st.pyplot(fig) # streamlit
#-----------------------------------------------------------------------------------------
def graph2(testdrive, df, incident_dict, map):
    st.markdown("Incident locations")
    plt.figure(figsize=(8,6), dpi=200)
    # plt.title('Plot 2')
    plt.xlabel("X coordinates")
    plt.ylabel("Y coordinates")
    for index,row in map.iterrows():
        x = row['x']
        y = row['y']
        c=plt.Circle((x, y),5,color = color5,edgecolor="black", linewidth=5)
        plt.gca().add_artist(c)
    plt.scatter(df['pose.position.x'],df['pose.position.y'],color = color1,s = 4,label = 'Route')
    plt.scatter([],[], color= 'violet', label = 'Station')
    plt.scatter(float(list(df['pose.position.x'])[0]),float(list(df['pose.position.y'])[0]),color = 'yellow', edgecolors="black",s = 200,marker = '^',label = 'Start') 
    loc1 = incident_dict[testdrive]['loc1']
    loc2 = incident_dict[testdrive]['loc2']
    plt.scatter(df.iloc[loc1]['pose.position.x'],df.iloc[loc1]['pose.position.y'],s=200,color = 'red',marker = '*',edgecolors = 'black',label = 'Condition1')
    plt.scatter(df.iloc[loc2]['pose.position.x'],df.iloc[loc2]['pose.position.y'],s=200,color = 'blue',marker = 'x',edgecolors = 'black', label = 'Condition2')
    plt.legend(bbox_to_anchor=(0.5, 1.2), loc='upper center', ncol = 3, fancybox = True, labelspacing = 1)
    plt.style.use('seaborn-whitegrid')
    st.pyplot(plt) # streamlit
#-----------------------------------------------------------------------------------------
def percent_mode(df):
    st.markdown("Autonomous Percentage")
    plt.figure(figsize=(4,4), dpi=200)
    df['delta_x'] = df['pose.position.x'].diff()
    df['delta_y'] = df['pose.position.y'].diff()
    df['distance'] = (df['delta_x']**2 + df['delta_y']**2)**0.5
    auto = df[df['msg.mode'] == 'True']['distance'].sum(axis=0)
    d = df['distance'].sum(axis=0)
    auto = round(auto,2); d = round(d,2); manu = round(d-auto,2)
    plt.pie([auto,manu], labels=['Autonomous','Manual'], autopct='%1.1f%%',colors=[color3,color4])
    st.pyplot(plt) # streamlit

#-----------------------------------------------------------------------------------------
def main():
    info, incident_dict = information()
    n = len(subfolders)
    mileages_stat(info,n)
    st.markdown('##')
    with st.sidebar:
        selected = st.selectbox('Select testdrive',tuple(subfolders))
        st.write('Note: Some test drives may have insufficient data to generate plot')
    st.header(f'Viewing {selected}')
    df, starttime = loaddb(selected)
    st.markdown('#')
    boxes(selected, info)
    df2, map = readfile(selected, df)
    testdrive = selected
    st.markdown('#')
    q1,q2,q3,q4 = st.columns(4,gap = 'large')
    with q1:
        plotg(df,'Velocity_x', 'twist.linear.x', 'Velocity(m/s)')
    with q2:
        plotg(df,'Acceleration_x', 'linear_acceleration.x_filtered', 'Acceleration(m/s^2)')
    with q3:
        df['percent_brake'] = df['msg.brake']/250*100
        plotg(df,'Brake Input', 'percent_brake', 'Brake%')
    with q4:
        plotg(df,'Distance', 'mileages', 'Distance(m)')
    st.markdown('#')
    q1,q2,q3= st.columns(3,gap = 'large')
    with q1:
        graph1(testdrive, df, df2, incident_dict, starttime)
    with q2:
        graph2(testdrive, df, incident_dict, map)
    with q3:
        percent_mode(df)
    
    @st.cache_data 
    def convert_df(df):
        # IMPORTANT: Cache the conversion to prevent computation on every rerun
        return df.to_csv(index=False).encode('utf-8')
    csv = convert_df(df)
    st.download_button(label="Download CSV", data=csv, file_name=f'{testdrive}.csv', mime='csv')
    
#-----------------------------------------------------------------------------------------
main()

url = 'https://seniorproject-incident-query.streamlit.app/'
st.write(f"Go to Incident Query [link]({url})")
