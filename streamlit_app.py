import pandas as pd
import matplotlib.pyplot as plt
import pymongo
from pymongo import MongoClient
import streamlit as st


# RETRIEVE DATA FROM DB
connect = 'mongodb+srv://kopkap:kopkap123@cluster0.agjmc4n.mongodb.net/?retryWrites=true&w=majority' # Atlas
# connect = 'mongodb://localhost:27017' # Compass 
cluster = MongoClient(connect)
db = cluster["Senior_project"]

testdrive = st.sidebar.selectbox("Select Testdrive",tuple(db.list_collection_names()))
collection = db[testdrive]
df = collection.find()
df = pd.DataFrame(df)
df = df.drop(columns = ['_id','Unnamed: 0'])
st.write(df)
