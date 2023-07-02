import pandas as pd
import matplotlib.pyplot as plt
import pymongo
from pymongo import MongoClient
import streamlit as st
import numpy as np
import os
import webbrowser
import base64
from collections import Counter
from  itertools import chain
from datetime import datetime

color1 = 'navy'
color2 = 'darkorange'
# RETRIEVE DATA FROM DB
connect = 'mongodb+srv://kopkap:kopkap123@cluster0.agjmc4n.mongodb.net/?retryWrites=true&w=majority' # Atlas
# connect = 'mongodb://localhost:27017' # Compass 
cluster = MongoClient(connect)
db = cluster["video"]
db2 = cluster["Testdrive"]
collection_list = db.list_collection_names()
collection = db['incidents']
#-------------------------------------------------------------------------------------------------
def select_month(df):
    df['month'] = df['date'].astype('datetime64').apply(lambda x: x.strftime('%Y-%m'))
    list_m = list(df['month'].unique())
    return df, st.selectbox('Month',tuple(list_m))
#-------------------------------------------------------------------------------------------------
def plot_month(df,input_month):
    gb = df.groupby(['month'])
    df2 = gb.get_group(input_month)

    fig, ax = plt.subplots(figsize=(6,6), dpi=200)
    barchart(df, 'Total', fig, ax, color1, True)
    barchart(df2, input_month, fig, ax, color2, False)
    ax.set_xlabel("Amount") 
    ax.set_ylabel("Tags") 
    ax.grid(True)
    fig.legend(bbox_to_anchor=(0.5, 0.9), loc='lower center', ncol = 3, fancybox = True)
    st.write('Existing Tags')
    st.pyplot(fig)
#-------------------------------------------------------------------------------------------------
def barchart(df,label,fig,ax,color,marker):
    df = pd.Series(Counter(chain(*df['tags']))).sort_index().rename_axis('tags').reset_index(name='freq')
    ax.barh(df['tags'], df['freq'], color = color, label = label,align='center',height=0.5)
    if marker:
        for i, v in enumerate(df['freq']):
            ax.text(float(v)+0.05 , float(i)-0.1, str(v), color='black', fontweight='light')
#-------------------------------------------------------------------------------------------------
def incident_numbers():
    all_testdrive = db2.list_collection_names()
    temp = {i:list(df['testdrive']).count(i) for i in all_testdrive}
    # st.write(temp)
    fig, ax = plt.subplots(figsize=(6,6), dpi =200)
    ax.barh(list(temp.keys()), list(temp.values()), color = color1 ,align='center',height=0.5, label = 'Incidents')
    for i, v in enumerate(list(temp.values())):
            ax.text(float(v)+0.05 , float(i)-0.1, str(v), color='black', fontweight='light')
    ax.set_xlabel("Incidents") 
    ax.set_ylabel("Test drives") 
    ax.grid(True)
    fig.legend(bbox_to_anchor=(0.5, 0.9), loc='lower center', ncol = 3, fancybox = True)
    st.write('Numbers of Incident Videos')
    st.pyplot(fig)
#===========================================================================================================================
# STREAMLIT
st.set_page_config(page_title="Senior Project", page_icon="*", layout="wide")
st.header('Incident Query')
st.markdown('#')
df = collection.find()
df = pd.DataFrame(df)
df = df[df['tags'] != ""]
df.index = np.arange(1, len(df) + 1)
col1,col2 = st.columns(2)
col1.metric('Total Records',str(len(df)))
# st.write(f'{len(df)} Total incidents')
st.write(df.drop(columns = ['_id','path','index']))
#--------------------------------------------------
factors = ['External','Internal']
# EXTERNAL
actors = ['Vehicle','Truck','Tuk-Tuk','Bus','Motorcycle','Pedestrian','Animal','Cyclist','Scooter','Non-Motor Vehicle','Other Vehicle']
environments = ['Obstacle','Straight Road','Slope','Corner','Crosswalk','Junction','Roundabout','Speed Bump']
scenarios = ['Cut In','Parking','Emergency Brake','Red Light Running','Wrong-Way Driving','Turning Across Lane','Other Scenario']
# INTERNAL
occupants = ['Driver','Passenger','Emergency']
systems = ['Battery','Sensing','Localization','Planning','Computing Node']

with st.sidebar:
    st.write('Select Tags')
    factor = st.multiselect('Factors',tuple(factors))
    actor = st.multiselect('Actors',tuple(actors))
    environment = st.multiselect('Environments',tuple(environments))
    scenario = st.multiselect('Scenarios',tuple(scenarios))
    occupant = st.multiselect('Occupants',tuple(occupants))
    system = st.multiselect('Systems',tuple(systems))
    tags = list(set(factor + actor + environment + occupant + system + scenario))
    # print(tags)
#--------------------------------------------------
st.markdown('#')
st.write('List of Tags')
st.image('Tags.png')

df, input_month = select_month(df)
q1,q2 = st.columns(2,gap='large')
with q2:
    incident_numbers()
with q1:
    plot_month(df,input_month)

#-------------------------------------------------------------------------------------------------
# QUERY
def data_query():
    key = 'tags'
    value = tags
    queries = collection.find({key:{"$all":value}})
    df_i = pd.DataFrame(queries)
    df_i.index = np.arange(1, len(df_i) + 1)
    st.markdown('#')
    st.header('Results')
    st.write(df_i[['stamp','testdrive','tags','url']])
    selected = st.selectbox('Select incident number',range(1,len(df_i)+1))-1
    path = df_i.iloc[selected ,df.columns.get_loc('path')]
    URL = df_i.iloc[selected,df.columns.get_loc('url')]
    testdrive = df_i.iloc[selected ,df.columns.get_loc('testdrive')]
    st.write('Video(drive)')
    st.write(URL)

    collection2 = db2[testdrive]
    df2 = collection2.find()
    df2 = pd.DataFrame(df2)
    df2 = df2.drop(columns = ['_id','Unnamed: 0'])
    csv = df2.to_csv(index=False)
    b64 = base64.b64encode(csv.encode()).decode()
    href = f'<a href="data:file/csv;base64,{b64}" download="{testdrive}.csv">Download CSV file of {testdrive}</a>'
    st.write('CSV(db)')
    st.markdown(href, unsafe_allow_html=True)       
    #-------------------------------------------------------------------------------------------------
if tags !=[]:
    data_query()

