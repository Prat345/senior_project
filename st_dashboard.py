import pandas as pd
import matplotlib.pyplot as plt
import pymongo
from pymongo import MongoClient
import streamlit as st
import numpy as np
import base64
from datetime import datetime
import matplotlib.dates as mdates
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objs as go

# st.cache(hash_funcs={"MyUnhashableClass": lambda _: None})
st.set_page_config(page_title="Senior Project", page_icon="*", layout="wide")
st.title("Test drive Dashboard")
st.markdown('#')

color1 = 'teal'
color2 = 'darkorange'
color3 = 'royalblue' # auto
color4 = 'limegreen' # manual
color5 = 'violet'
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
def sort_by(df, var, title):
    df['d_auto'] = df['mileages']*df['p_auto']/100
    by_var = df.groupby(var)
    variables = by_var.groups.keys()
    by_var = by_var.sum()
    by_var['p_auto'] = by_var['d_auto']/by_var['mileages']*100
    # st.write(by_var)
    mileages_list = by_var['mileages']
    auto_list = by_var['p_auto']
    manual_list = 100-by_var['p_auto']

    fig, ax1 = plt.subplots(figsize=(5,4), dpi=200)
    ax2 = ax1.twinx()
    ax1.set_ylim(0,round(max(mileages_list),-3)+4000)
    ax2.set_ylim(0,100)
    barwidth = 0.4
    ax1.plot(variables, mileages_list, color = color2, lw = 3, marker = '^',markeredgecolor = 'black',label = 'Total Mileages')
    # for i, v in enumerate(mileages_list):
    #     ax1.text(float(i)-.3 , float(v)+300, str(round(v))+' m', color='black', fontweight='bold')
    #----------------------------------------------------
    ax2.bar(variables, auto_list, width = barwidth, color=color3, label = 'Auto%')
    ax2.bar(variables, manual_list, bottom = auto_list, width = barwidth, color=color4, label = 'Manual%')
    for i, v in enumerate(auto_list):
        ax2.text(float(i)-.15 , float(v)/2, str(round(v,2))+'%', color='white', fontweight='light')
        ax2.text(float(i)-.15 , float(manual_list[i])/2+float(v), str(round(manual_list[i],2))+ '%', color='white', fontweight='light')
    #----------------------------------------------------
    ax1.set_ylabel(r"Mileages $(m)$", color=color2)
    for label in ax1.get_yticklabels():
        label.set_color(color2)
    ax1.set_xlabel(title) 
    ax2.set_ylabel("Autonomous (%)", color=color1)
    for label in ax2.get_yticklabels():
        label.set_color(color1)
    fig.legend(bbox_to_anchor=(0.5, 0.9), loc='lower center', ncol = 3, fancybox = True)
    ax1.grid(False)
    ax2.grid(True)
    st.markdown(f'Mileages by {title}')
    ax1.set_zorder(ax2.get_zorder()+1)
    ax1.patch.set_visible(False)
    st.pyplot(plt) # streamlit
    
#-----------------------------------------------------------------------------------------
def mileages_stat(df,n):
    total_mileages = round(df['mileages'].sum(axis=0),2)
    d_auto = df['mileages'] * df['p_auto']/100
    p_auto = d_auto.sum(axis=0)/total_mileages * 100
    p_auto = round(p_auto,2)
    col1, col2, col3 = st.columns(3)
    col1.metric('Total Test drives',str(n), delta=+2, delta_color="normal")
    col2.metric('Total Mileages',str(total_mileages) + ' m', delta=+2512.07, delta_color="normal")
    col3.metric('Average Autonomous %',str(p_auto)+ '%', delta=-3.71, delta_color="normal")

    q1,q2= st.columns(2,gap = 'large')
    with q1:
        sort_by(df,'vehicle','Vehicles')
    with q2:
        df['month'] = pd.to_datetime(df['date']).apply(lambda date: date.strftime('%y-%m'))
        sort_by(df,'month','Months')
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
def pie_chart(df):
    st.markdown("Autonomous Percentage")
    df['delta_x'] = df['pose.position.x'].diff()
    df['delta_y'] = df['pose.position.y'].diff()
    df['distance'] = (df['delta_x']**2 + df['delta_y']**2)**0.5
    auto = df[df['msg.mode'] == 'True']['distance'].sum(axis=0)
    d = df['distance'].sum(axis=0)
    auto = round(auto,2); d = round(d,2); manu = round(d-auto,2)

    fig = px.pie(names=['Auto','Manual'],values=[auto,manu], height=300, width=300, hole=0.7, 
                    color_discrete_sequence=[color3,color4])
    fig.update_traces(hovertemplate=None, textposition='outside',textinfo='percent+label', rotation=50)
    fig.update_layout(margin=dict(t=50, b=35, l=0, r=0), showlegend=False,
                            font=dict(size=17, color='#8a8d93'),
                            hoverlabel=dict(bgcolor="#444", font_size=10, font_family="Lato, sans-serif"))
    fig.add_annotation(dict(x=0.5, y=0.5,  align='center',
                            xref = "paper", yref = "paper",
                            showarrow = False, font_size=20,
                            text="Drive Mode"))
    st.plotly_chart(fig)
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
        plotg(df,'Velocity_x', 'twist.linear.x', r'Velocity &(m/s)&')
    with q2:
        plotg(df,'Acceleration_x', 'linear_acceleration.x_filtered', r'Acceleration $(m/s^2)$')
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
        pie_chart(df)
    
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
