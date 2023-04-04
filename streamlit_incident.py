import pandas as pd
import matplotlib.pyplot as plt
import pymongo
from pymongo import MongoClient
import streamlit as st
import numpy as np
import os
import webbrowser

# RETRIEVE DATA FROM DB
connect = 'mongodb+srv://kopkap:kopkap123@cluster0.agjmc4n.mongodb.net/?retryWrites=true&w=majority' # Atlas
# connect = 'mongodb://localhost:27017' # Compass 
cluster = MongoClient(connect)
db = cluster["video"]
db2 = cluster["Testdrive"]
collection_list = db.list_collection_names()
collection = db['incidents']

st.header('Incident Search')
st.markdown('#')
df = collection.find()
df = pd.DataFrame(df)
st.write(df.drop(columns = ['_id']))

st.markdown('#')
st.header('Select Tags')
factors = ['External','Internal']
# EXTERNAL
actors = ['Vehicle','Truck','Tuk-Tuk','Bus','Motorcycle','Pedestrian','Animal','Cyclist','Scooter','Non-Motor Vehicle','Other Vehicle']
environments = ['Obstacle','Straight Road','Slope','Corner','Crosswalk','Junction','Roundabout','Speed Bump']
scenarios = ['Cut In','Overtaking','Parking','Emergency Brake','Red Light Running','Wrong-Way Driving','Turning Across Lane','Other Scenario']
# INTERNAL
occupants = ['Driver','Passenger','Emergency']
systems = ['Battery','Sensing','Localization','Planning','Computing Node']

factor = st.multiselect('Factors',tuple(factors))
actor = st.multiselect('Actors',tuple(actors))
environment = st.multiselect('Environments',tuple(environments))
scenario = st.multiselect('Scenarios',tuple(scenarios))
occupant = st.multiselect('Occupants',tuple(occupants))
system = st.multiselect('Systems',tuple(systems))
tags = list(set(factor + actor + environment + occupant + system))
print(tags)
#-------------------------------------------------------------------------------------------------
# QUERY
def data_query():
    key = 'tags'
    value = tags
    queries = collection.find({key:{"$all":value}})
    df_i = pd.DataFrame(queries)
    st.markdown('#')
    st.write('Results')
    st.write(df_i)
    selected = st.selectbox('Select incident to view',tuple(list(np.arange(len(df_i)))))
    path = df_i.iloc[selected ,df.columns.get_loc('path')]
    URL = df_i.iloc[selected ,df.columns.get_loc('url')]
    testdrive = df_i.iloc[selected ,df.columns.get_loc('testdrive')]
    st.write(URL)
    q1,q2,q3,q4= st.columns(4)
    with q1:
        if st.button('Video(local)'):
            os.startfile(path)
    with q2:
        if st.button('Video(drive)'):
            webbrowser.open(URL)
    with q3:
        if st.button('CSV(db)'):
            collection2 = db2[testdrive]
            df2 = collection2.find()
            df2 = pd.DataFrame(df2)
            df2 = df2.drop(columns = ['_id','Unnamed: 0'])
            df2.to_csv(f'{testdrive}.csv')
            os.startfile(f'{testdrive}.csv')

    #-------------------------------------------------------------------------------------------------
if tags !=[]:
    data_query()

