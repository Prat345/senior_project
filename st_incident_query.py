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

# RETRIEVE DATA FROM DB
connect = 'mongodb+srv://kopkap:kopkap123@cluster0.agjmc4n.mongodb.net/?retryWrites=true&w=majority' # Atlas
# connect = 'mongodb://localhost:27017' # Compass 
cluster = MongoClient(connect)
db = cluster["video"]
db2 = cluster["Testdrive"]
collection_list = db.list_collection_names()
collection = db['incidents']
#-------------------------------------------------------------------------------------------------
def monthly(df):
    plt.figure()
    month = df
    month['month'] = month['date'].astype('datetime64').apply(lambda x: x.strftime('%Y-%m'))
    # st.write(month)
    month2 = pd.Series(Counter(month['month'])).sort_index().rename_axis('month').reset_index(name='freq')
    # st.write(month2)
    # plt.barh(month2['month'], month2['freq'], color = 'navy', label = 'Total Mileages',align='center',height=0.5)
    # st.pyplot(plt)
    list_m = list(set(month['month']))
    select_month= st.selectbox('View by each month',tuple(list_m))
    gb = month.groupby(['month'])
    df3 = gb.get_group(select_month)
    # df3['tags'] = df3['tags'].str.strip('[]').str.replace("'",'').str.split(',') # Convert string to list
    q1,q2 = st.columns(2)
    with q1:
        st.write(f'{len(df3)} incidents')
        st.write(df3['tags'])
    with q2:
        barchart(df3)
#-------------------------------------------------------------------------------------------------
def barchart(df):
    plt.figure()
    df2 = pd.Series(Counter(chain(*df['tags']))).sort_index().rename_axis('tags').reset_index(name='freq')
    # st.write(df2)
    plt.barh(df2['tags'], df2['freq'], color = 'royalblue', label = 'Tags',align='center',height=0.5)
    for i, v in enumerate(df2['freq']):
        plt.text(float(v)+0.05 , float(i)-0.1, str(v), color='black', fontweight='light')
    plt.xlabel("Amount") 
    plt.ylabel("Tags") 
    plt.style.use('seaborn-whitegrid')
    st.write('Existing Tags')
    st.pyplot(plt)
#-------------------------------------------------------------------------------------------------
def incident_numbers():
    all_testdrive = db2.list_collection_names()
    temp = {i:list(df['testdrive']).count(i) for i in all_testdrive}
    # st.write(temp)
    plt.figure()
    plt.barh(list(temp.keys()), list(temp.values()), color = 'royalblue',align='center',height=0.5)
    for i, v in enumerate(list(temp.values())):
            plt.text(float(v)+0.05 , float(i)-0.1, str(v), color='black', fontweight='light')
    plt.xlabel("Incidents") 
    plt.ylabel("Test drives") 
    plt.style.use('seaborn-whitegrid')
    st.write('Numbers of Incident Videos')
    st.pyplot(plt)
#-------------------------------------------------------------------------------------------------
# STREAMLIT
st.set_page_config(page_title="Senior Project", page_icon="*", layout="wide")
st.header('Incident Search')
st.markdown('#')
df = collection.find()
df = pd.DataFrame(df)
df = df[df['tags'] != ""]
df.index = np.arange(1, len(df) + 1)
col1,col2 = st.columns(2)
col1.metric('Total incident videos',str(len(df)))
# st.write(f'{len(df)} Total incidents')
st.write(df.drop(columns = ['_id']))
#--------------------------------------------------
factors = ['External','Internal']
# EXTERNAL
actors = ['Vehicle','Truck','Tuk-Tuk','Bus','Motorcycle','Pedestrian','Animal','Cyclist','Scooter','Non-Motor Vehicle','Other Vehicle']
environments = ['Obstacle','Straight Road','Slope','Corner','Crosswalk','Junction','Roundabout','Speed Bump']
scenarios = ['Cut In','Parking','Emergency Brake','Red Light Running','Wrong-Way Driving','Turning Across Lane','Other Scenario']
# INTERNAL
occupants = ['Driver','Passenger','Emergency']
systems = ['Battery','Sensing','Localization','Planning','Computing Node']
#--------------------------------------------------
st.markdown('#')
st.write('List of Tags')
st.image('Tags.png')

st.markdown('#')
q1,q2 = st.columns(2)
with q1:
    barchart(df)
with q2:
    incident_numbers()
st.markdown('#')
st.markdown('#')
monthly(df)

st.markdown('#')
st.write('Select Tags')
factor = st.multiselect('Factors',tuple(factors))
actor = st.multiselect('Actors',tuple(actors))
environment = st.multiselect('Environments',tuple(environments))
scenario = st.multiselect('Scenarios',tuple(scenarios))
occupant = st.multiselect('Occupants',tuple(occupants))
system = st.multiselect('Systems',tuple(systems))
tags = list(set(factor + actor + environment + occupant + system + scenario))
print(tags)
#-------------------------------------------------------------------------------------------------
# QUERY
def data_query():
    key = 'tags'
    value = tags
    queries = collection.find({key:{"$all":value}})
    df_i = pd.DataFrame(queries)
    df_i.index = np.arange(1, len(df_i) + 1)
    st.markdown('#')
    st.write('Results')
    st.write(df_i[['stamp','testdrive','tags','url']])
    selected = st.selectbox('Select incident to view',range(1,len(df_i)+1))-1
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

