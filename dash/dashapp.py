from dash import Dash, html, dcc, Output, Input  # pip install dash
import dash_bootstrap_components as dbc    # pip install dash-bootstrap-components
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd                       
import numpy as np
import os
import pymongo

color1 = '#4da6ff' # auto
color2 = '#8cff66' # manual
color3 = 'teal'

connect = 'mongodb+srv://kopkap:kopkap123@cluster0.agjmc4n.mongodb.net/?retryWrites=true&w=majority' # Atlas
# connect = 'mongodb://localhost:27017' # Compass 
cluster = pymongo.MongoClient(connect)
db1 = cluster["Testdrive"]
testdrives = db1.list_collection_names()

db2 = cluster["Information"]
incident_col = db2['Incident']
incident_dict = incident_col.find_one()

info_col = db2['Testdrive_info']
info = info_col.find()
info = pd.DataFrame(info, dtype = 'float64') 
info.drop(columns = ['_id','Unnamed: 0'],inplace=True)
info_sort = info.sort_values(by=['datetime'])
avg_auto = (info['p_auto']*info['mileages']).sum()/info['mileages'].sum()
avg_auto = round(avg_auto,1)
# testdrives = os.listdir('D:\FOOH\Senior_Project/testdrives')
# info = pd.read_csv(f'D:\FOOH\Senior_Project/testdrive_info.csv')
map = pd.read_csv('station.csv', index_col=['location','vehicle','index'], skipinitialspace=True)
topics = ['twist.linear.x','linear_acceleration.x_filtered', 'msg.brake', 'mileages']

fig0 = px.bar(info.dropna(subset='mileages'), x='testdrive', y='mileages',
             hover_data=['p_auto', 'p_manual'], color='p_auto',
             labels={'pop':'population of Canada'}, height=300, 
             text_auto='.2s',
             color_continuous_scale='viridis_r')
fig0.update_traces(textfont_size=12, textangle=0, textposition="outside", cliponaxis=False)
fig0.update_layout(margin=dict(l=0, r=10, t=0, b=0),
                   yaxis_range=[0, max(info['mileages'])*1.2])

#-------------------------------------------------------------------------------------------------------
# Components
app = Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP],
            meta_tags=[{'name':' viewport', 'content': 'width=device-width, initial-scale=0.1'}])
server = app.server
bar_testdrive = dcc.Graph(figure=fig0)
pie_mileage = dcc.Graph(id='pie1',figure={})
hist_mode = dcc.Graph(id='hist1',figure={})

testdrive_name = dcc.Markdown(children='Viewing '+ 'T2-B_SL_NBTC_20230125202407')
para_chart = dcc.Graph(id='para_chart',figure={})
wp_chart = dcc.Graph(id='wp_chart',figure={})
dropdown = dcc.Dropdown(options=testdrives,
                        value='T2-B_SL_NBTC_20230125202407',  # initial value displayed when page first loads
                        clearable=False)
dropdown2 = dcc.Dropdown(options=topics,
                        value=topics[1],  
                        clearable=False)
dropdown3 = dcc.Dropdown(options=['month','vehicle'],
                        value='vehicle', 
                        clearable=False)
#-------------------------------------------------------------------------------------------------------
# Page Layout
app.layout = dbc.Container([
    dbc.Row([
        dbc.Col([dcc.Markdown('# Incident Dashboard')], width={'size': 6, 'offset': 0})
    ], className="mb-2", justify='left'),

    dbc.Row([
        dbc.Col([
            dbc.Row([
                dbc.Col([
                    dbc.Card([
                        dbc.CardBody([
                            html.P("Test drives"),
                            html.H1(f'{len(testdrives)}', className='text-success')])
                                ],className="text-center border-0 shadow")
                        ], width=4),

                dbc.Col([
                    dbc.Card([
                        dbc.CardBody([
                            html.P("Avg Auto Mode"),
                            html.H1(f'{avg_auto} %', className='text-info')])
                                ],className="text-center border-0 shadow")
                        ], width=4),
                
                dbc.Col([
                    dbc.Card([
                        dbc.CardBody([
                            html.P("Incidents"),
                            html.H1('11', className='text-warning')])
                                ],className="text-center border-0 shadow")
                        ], width=4)

                    ], className="mb-2", justify='center'),
                    
            dbc.Row([
                dbc.Col([
                    dbc.Card([
                        dbc.CardBody([
                            html.P("Drive mode"),
                            hist_mode # *********
                                ])
                            ],className ="justify-content-center border-0 shadow")
                        ], width=7),

                dbc.Col([
                    dbc.Row([
                        dbc.Col([dropdown3], width=12)
                            ], className="mb-2", justify='left'),
                    dbc.Row([
                        dbc.Col([
                            dbc.Card([
                                dbc.CardBody([
                                    html.P("Mileages"),
                                    pie_mileage # *********
                                        ])
                                    ],className ="justify-content-center border-0 shadow")
                                ], width=12)
                            ])
                        ], width=5)
                    ], className="mb-2", justify='left'),

            dbc.Row([
                dbc.Col([
                    dbc.Card([
                        dbc.CardBody([
                            html.P("Test drives"),
                            bar_testdrive # *********
                                ])
                            ], className ="justify-content-center border-0 shadow")
                        ], width=12)
                    ], className="mb-2", justify='center')
        ], width=6),

        dbc.Col([
            dbc.Row([
                dbc.Col([testdrive_name], width=12)
                    ], className="mb-0", justify='left'),

            dbc.Row([
                dbc.Col([dropdown], width=6),
                dbc.Col([dropdown2], width=4)
                    ], className="mb-2", justify='left'),

            dbc.Row([
                dbc.Col([
                        dbc.Card([
                        dbc.CardBody([
                            html.P("Incident Timeframe"),
                            para_chart # *********
                                    ])
                                ], className ="justify-content-center border-0 shadow")
                        ], width=12)
                    ], className="mb-2", justify='left'),
            dbc.Row([
                dbc.Col([
                        dbc.Card([
                        dbc.CardBody([
                            html.P("Incident Location"),
                            wp_chart # *********
                                    ])
                                ], className ="justify-content-center border-0 shadow")
                        ], width=12)
                    ], className="mb-2", justify='center')
        ], width=6)

    ],className="mt-2", justify='center'),
    # dcc.Store inside the user's current browser session
    dcc.Store(id='store-data', data=[], storage_type='memory'),
    dcc.Store(id='store-data2', data=[], storage_type='memory'),  # 'local' or 'session'
], fluid=True, style={'backgroundColor':'#F5F5F5'})
#-------------------------------------------------------------------------------------------------------
# Callback for interactive components
@app.callback(
    Output(testdrive_name, 'children'), # (component_id, component_property)
    Output('store-data', 'data'),
    Output('store-data2', 'data'),
    Input(dropdown, 'value')
)
def get_data(testdrive):  # function arguments come from the component property of the Input
    # print(testdrive)
    collection = db1[testdrive]
    df = collection.find()
    df = pd.DataFrame(df, dtype = 'float64')
    df.drop(columns = ['_id','Unnamed: 0'], inplace=True)
    position = df.columns.get_loc('Time')
    starttime = df.iat[0, position]
    df['elasped_sec'] = (df.iloc[1:, position] - starttime)
    df['elasped_sec'].fillna(0.000,inplace = True)
    df['elasped_time'] = pd.to_datetime(df['elasped_sec'],unit = 's')
    df.iloc[:,1:] = df.iloc[:,1:].round(3)
    df['elasped_time'] = df['elasped_time'].apply(lambda dt: dt.strftime('%H:%M:%S'))
    df['msg.mode'] = df['msg.mode'].map({'True': True, 'False': False})
    print(f'\n>>>>>> {testdrive}')
    print(df.head())
    print(incident_dict[testdrive])
    return 'Viewing '+ testdrive, df.to_dict('records'), incident_dict[testdrive] # returned objects are assigned to the Output

@app.callback(
    Output('hist1', 'figure'),
    Output('pie1', 'figure'),
    Input(dropdown3, 'value')
)
def mileage_chart(var):
    info['d_auto'] = info['mileages']*info['p_auto']/100
    by_var = info.groupby(var)
    by_var = by_var.sum(numeric_only=True)
    by_var['p_auto'] = by_var['d_auto']/by_var['mileages']*100
    by_var = by_var.round(0).sort_index()
    total_mil = round(by_var['mileages'].sum())
    auto_list = by_var['p_auto']
    manual_list = 100-by_var['p_auto']
    index = by_var.index
    fig = go.Figure()
    fig.add_trace(go.Bar(x=index, y=auto_list, name='Auto',
                        text=[f'{i}%' for i in auto_list],
                        width = 0.5,
                        marker_color='royalblue'))
    fig.add_trace(go.Bar(x=index, y=manual_list, name='Manual',
                        text=[f'{i}%' for i in manual_list],
                        width = 0.5,
                        marker_color='limegreen'))
    fig.update_layout(margin=dict(l=0, r=10, t=0, b=0),
                      height=245,
                      paper_bgcolor="white",
                      barmode='stack',
                      legend=dict(
                            orientation="h",
                            yanchor="bottom",
                            y=1.02,
                            xanchor="left",
                            x=0.1
                            ))
    fig.update_traces(textfont_size=10, textangle=0, textposition="inside", cliponaxis=False)

    fig2 = px.pie(names=index,values=by_var['mileages'], height=200, hole=0.7,
                  color_discrete_sequence=px.colors.sequential.RdBu_r)
    fig2.update_traces(hovertemplate=None, textposition='outside',
                       textinfo='label', rotation=180,
                    #    marker=dict(colors=['darkblue', 'royalblue','lightcyan','cyan'])
                       )
    fig2.update_layout(margin=dict(l=0, r=0, t=0, b=0), showlegend=False)
    fig2.add_annotation(dict(x=0.5, y=0.5,  align='center',
                            xref = "paper", yref = "paper",
                            showarrow = False, font_size=20,
                            text=f'{str(total_mil)} m'))
    return fig, fig2

@app.callback(
    Output('para_chart', 'figure'),
    Input('store-data', 'data'),
    Input('store-data2', 'data'),
    Input(dropdown2, 'value')
)
def parameter_chart(data,incident_d,topic):
    df = pd.DataFrame(data)
    df['elasped_time'] = df['elasped_time'].apply(lambda t: t[3:])
    # print(incident_d)

    t_con1 = []
    t_con2 = []
    if len(incident_d['loc1']) > 0:
        con1 = pd.Series(incident_d['loc1'])
        t_con1 = df.iloc[con1]['elasped_time']
    if len(incident_d['loc2']) > 0:
        con2 = pd.Series(incident_d['loc2'])
        t_con2 = df.iloc[con2]['elasped_time']
    # con1 = pd.Series(incident_d['loc1'])
    # con2 = pd.Series(incident_d['loc2'])
    # t_con1 = df.iloc[con1]['elasped_time']
    # t_con2 = df.iloc[con2]['elasped_time']

    fig = make_subplots(rows=3, cols=1, shared_xaxes=True, row_heights=[0.1,0.8,0.1])
    if topic == 'mileages':
        fig.add_trace(go.Scatter(x=df['elasped_time'], 
                         y=df[topic],
                         mode='lines',
                         fill='tozeroy',  
                         name=topic
                         ),
                    row=2,col=1)
    else:
        fig.add_trace(go.Scatter(x=df['elasped_time'], 
                         y=df[topic],
                         mode='lines',
                         name=topic
                         ),
                    row=2,col=1)
    fig.add_trace(go.Scatter(mode='markers',x=t_con1,
                            y=np.zeros(len(df)),
                            marker_symbol='line-ns',
                            marker_line_color="red", opacity=1,
                            marker_line_width=2, marker_size=20,
                            name='Condition 1'
                            ),
                        row=3,col=1)
    fig.add_trace(go.Scatter(mode='markers',x=t_con2,
                            y=np.zeros(len(df)),
                            marker_symbol='line-ns',
                            marker_line_color="darkorange", opacity=1,
                            marker_line_width=2, marker_size=20,
                            name='Condition 2'
                            ),
                        row=3,col=1)
    fig.add_trace(go.Scatter(mode='markers',x=df[df['msg.mode']==True]['elasped_time'], 
                         y=np.zeros(len(df)),
                         marker_symbol='line-ns',
                         marker_line_color=color1, opacity=1,
                         marker_line_width=2, marker_size=20,
                         name='Auto'
                         ),
                    row=1,col=1)
    fig.add_trace(go.Scatter(mode='markers',x=df[df['msg.mode']==False]['elasped_time'], 
                            y=np.zeros(len(df)),
                            marker_symbol='line-ns',
                            marker_line_color=color2, opacity=1,
                            marker_line_width=2, marker_size=20,
                            name='Manual'
                            ),
                    row=1,col=1)
    fig.update_yaxes(range=[-.5,.5],row=1, col=1)
    fig.update_yaxes(range=[-.5,.5],row=3, col=1)
    fig.update_layout(
                margin=dict(l=0, r=10, t=0, b=0),
                height=250,
                xaxis1=dict(
                    tickmode='linear',
                    tick0=0,
                    dtick=60 # 60 sec tick
                ),
                xaxis2=dict(
                    tickmode='linear',
                    tick0=0,
                    dtick=60
                ),
                xaxis3=dict(
                    tickmode='linear',
                    tick0=0,
                    dtick=60
                ),
                yaxis1=dict(
                    visible=False
                ),
                yaxis3=dict(
                    visible=False
                ),
                legend=dict(
                    orientation="h",
                    yanchor="bottom",
                    y=1.02,
                    xanchor="left",
                    x=0.1
                    ))
    fig.update_traces(line_color=color3,line_width=0.6)
    return fig

@app.callback(
    Output('wp_chart', 'figure'),
    Input('store-data', 'data'),
    Input('store-data2', 'data'),
    Input(dropdown, 'value')
)
def waypoint_chart(data,incident_d,testdrive):
    df = pd.DataFrame(data)
    testdrive = testdrive.replace('-','')
    testdrive = testdrive.replace('Chula','CU')
    vehicle,operator,location,date = testdrive.split('_')
    submap = map.loc[location].loc[vehicle]

    con1 = []
    con2 = []
    if len(incident_d['loc1']) > 0:
        con1 = pd.Series(incident_d['loc1'])
    if len(incident_d['loc2']) > 0:
        con2 = pd.Series(incident_d['loc2'])
    # con1 = pd.Series(incident_d['loc1'])
    # con2 = pd.Series(incident_d['loc2'])

    if 'NBTC' in testdrive.upper():
        tick = 20
    else:
        tick = 50  
    fig = go.Figure()
    fig.add_trace(go.Scatter(mode="markers", x=submap['x'], y=submap['y'], marker_symbol='circle',
                            marker_color='#FFCCFF', marker_line_color='black',
                            marker_line_width=0.5, opacity=1,
                            marker_size=50,
                            name='stations'
                            ))
    fig.add_trace(go.Scatter(x=df['pose.position.x'], y=df['pose.position.y'],mode='lines', name='Route'))
    fig.add_trace(go.Scatter(x=np.array(df['pose.position.x'][0]), y=np.array(df['pose.position.y'][0]), marker_symbol='triangle-up',
                            marker_line_color="black", marker_color="yellow",
                            marker_line_width=1, marker_size=15,name='start'
                            ))
    fig.add_trace(go.Scatter(mode="markers", x=df.iloc[con1]['pose.position.x'], y=df.iloc[con1]['pose.position.y'], marker_symbol='y-up',
                            marker_line_color="red", marker_color="red",
                            marker_line_width=2, marker_size=15,name='condition 1'
                            ))
    fig.add_trace(go.Scatter(mode="markers", x=df.iloc[con2]['pose.position.x'], y=df.iloc[con2]['pose.position.y'], marker_symbol='y-down',
                            marker_line_color="darkorange", marker_color="darkorange",
                            marker_line_width=2, marker_size=15,name='condition 2'
                            ))
    fig.update_layout(
                margin=dict(l=10, r=10, t=0, b=20),
                height=350,
                paper_bgcolor="white",
                xaxis=dict(tickmode='linear',tick0=0,dtick=tick),
                xaxis_range=[min(df['pose.position.x'])-tick,max(df['pose.position.x'])+tick],
                yaxis_range=[min(df['pose.position.y'])-tick,max(df['pose.position.y'])+tick],
                legend=dict(
                    orientation="h",
                    yanchor="bottom",
                    y=1.02,
                    xanchor="left",
                    x=0.1
                    )
                )
    fig.update_traces(line_color=color3,line_width = 3)
    return fig
#-------------------------------------------------------------------------------------------------------

# Run app
if __name__=='__main__':
    app.run_server(debug=True)

# ref
# https://dashcheatsheet.pythonanywhere.com/
# https://testdriven.io/blog/flask-render-deployment/
