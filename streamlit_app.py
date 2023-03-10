import pandas as pd
import matplotlib.pyplot as plt
import pymongo
from pymongo import MongoClient
import streamlit as st

st.set_page_config(page_title="Senior Project", page_icon="*", layout="wide")
st.title("Data of Autonomous Vehicles Testdrive")
st.markdown('#')

# RETRIEVE DATA FROM DB
connect = 'mongodb+srv://kopkap:kopkap123@cluster0.agjmc4n.mongodb.net/?retryWrites=true&w=majority' # Atlas
# connect = 'mongodb://localhost:27017' # Compass 
cluster = MongoClient(connect)
db1 = cluster["Senior_project"]
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
    st.header('Testdrives Inforamtion')
    st.write(info)
    return info,incident_dict
#-----------------------------------------------------------------------------------------
def readfile(testdrive, df):
    # with open(os.path.join(folder_path, testdrive +'/'+'waypoints_map.csv')) as f:
    #     map = pd.read_csv(f)
    testdrive = testdrive.replace('-','')
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
    df = info
    mileages_dict = {}
    gb = df.groupby(['vehicle'])
    for vehicle in ['T1','T2-B','T2-W']:
        mileages_dict[vehicle] = gb.get_group(vehicle)['mileages'].sum(axis=0)
    vehicles = list(mileages_dict.keys())
    mileages = list(mileages_dict.values())
    fig = plt.figure(figsize = (5,4))
    plt.bar(vehicles, mileages, color =['darkcyan','navy','lightskyblue'],width = 0.4)
    for i, v in enumerate(mileages):
        plt.text(float(i)-.2 , float(v)+100, str(round(v,2)), color='black', fontweight='light')
    plt.xlabel("Vehicle") 
    plt.ylabel("mileages(m)") 
    plt.style.use('seaborn-whitegrid')
    st.header('Mileages Statistics')
    st.pyplot(plt) # streamlit
    st.write('Total Mileages: ' + str(round(sum(mileages),2)) + ' meters')
#-----------------------------------------------------------------------------------------
def graph1(testdrive, df, df2, incident_dict):
    st.header("Figure 1: Incident Occurances")
    plt.figure(figsize=(5,4))
    # plt.title('Plot 1')
    plt.xlabel("Time")  
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
    st.pyplot(plt) # streamlit
#-----------------------------------------------------------------------------------------
def graph2(testdrive, df, incident_dict, map):
    st.header("Figure 2: Incident locations")
    plt.figure(figsize=(5,4))
    # plt.title('Plot 2')
    plt.xlabel("Time")
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
    st.header('Each Testdrive Data')
    selected = st.selectbox('Select testdrive',tuple(subfolders))
    st.markdown('#')
    df = loaddb(selected)
    st.header('Viewing: '+str(selected))
    boxes(selected, info)
    df2, map = readfile(selected, df)
    testdrive = selected
    st.markdown('#')
    graph1(testdrive, df, df2, incident_dict)
    st.markdown('#')
    graph2(testdrive, df, incident_dict, map)
    st.markdown('#')
    percent_mode(df)

#-----------------------------------------------------------------------------------------

main()
st.header('*'*70)
st.image('the-rock-eyebrow.gif')
