import streamlit as st
import pandas as pd
# import matplotlib.pyplot as plt
import plost

st.set_page_config(page_title="Senior Project", page_icon="o", layout="wide")

st.title("Data of autonomous vehicles")

file = open(r"D:\FOOH\Senior_Project\ublox-navpvt.csv")
df = pd.read_csv(file)
# print(df)

st.markdown('### Table')
st.write(df)
# fig, ax = plt.subplots(1,1)
# ax.scatter(x=df['Time'], y=df['gSpeed'])
# st.pyplot(fig)

# plot_height = st.sidebar.slider('Specify plot height', 200, 500, 250)
st.markdown('### Var 1')
st.line_chart(df, x = 'Time', y = 'gSpeed')
st.markdown('### Var 2')
st.line_chart(df, x = 'Time', y = 'vAcc')
st.markdown('### Var 3')
st.line_chart(df, x = 'Time', y = 'hAcc')
#    streamlit run D:\FOOH\Senior_Project\streamlit.py

st.write("Thank you")
