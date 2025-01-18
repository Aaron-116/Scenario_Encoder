import numpy as np
import pandas as pd
import csv
from data_adjust import data_adjustment

#确定egocar的搜索帧范围，如果搜索帧超出了ego的实际帧范围，则取ego的实际帧范围
def frame_range_actual(lower_frame, upper_frame, ego_track):
    ego_frame_range=ego_track['frame'].unique()
    # print(lower_frame, upper_frame, ego_frame_range)
    if lower_frame<=ego_frame_range[-1] and upper_frame>ego_frame_range[0]:
        ego_track_data = ego_track.loc[(ego_track['frame']<=lower_frame) & (ego_track['frame']>=upper_frame)]
    elif lower_frame>ego_frame_range[-1] and upper_frame>ego_frame_range[0]:
        ego_track_data = ego_track.loc[(ego_track['frame']<=ego_frame_range[-1]) & (ego_track['frame']>=upper_frame)]
    elif lower_frame<=ego_frame_range[-1] and upper_frame<=ego_frame_range[0]:
        ego_track_data = ego_track.loc[(ego_track['frame']<=lower_frame) & (ego_track['frame']>=ego_frame_range[0])]
    else:
        ego_track_data = ego_track.loc[(ego_track['frame']<=ego_frame_range[-1]) & (ego_track['frame']>=ego_frame_range[0])]

    ego_frame_range_actual=ego_track_data['frame'].unique()
    # print(ego_frame_range_actual)

    return ego_track_data, ego_frame_range_actual

def scenario_code(BV_id, ego_track_data, ego_laneId, BV_local_flag, end_frame, init_frame, df):
    scenario_code=9
    BV_data=df[df['ego_id']==BV_id]
    # print("BV_data长度",len(BV_data))
    # print(init_frame, end_frame)
    if BV_data.empty:
        scenario_code= 'None'
    else:
        BV_track_data, BV_frame_range_actual = frame_range_actual(end_frame,init_frame,BV_data)
        # print("BV长度",len(BV_frame_range_actual))
        # print(BV_frame_range_actual)
        if len(BV_frame_range_actual)==0:
            scenario_code= 'None'
        else:
            BV_init_frame=BV_frame_range_actual[0]
            BV_end_frame=BV_frame_range_actual[-1]
            BV_LC_inti_flag= BV_data[BV_data['frame']==BV_init_frame]['laneId'].values
            BV_LC_end_flag= BV_data[BV_data['frame']==BV_end_frame]['laneId'].values
            BV_lane_change_flag=BV_track_data['laneId'].unique()
            ego_init_frame_data=ego_track_data[ego_track_data['frame']==BV_init_frame]
            ego_end_frame_data=ego_track_data[ego_track_data['frame']==BV_end_frame]
            BV_init_frame_data=BV_track_data[BV_track_data['frame']==BV_init_frame]
            BV_end_frame_data=BV_track_data[BV_track_data['frame']==BV_end_frame]
                            
            # print(BV_init_frame, BV_end_frame)
            # print(BV_init_frame_data, BV_end_frame_data)
            # print(ego_init_frame_data, ego_end_frame_data)

            if BV_init_frame_data.empty or BV_end_frame_data.empty or ego_init_frame_data.empty or ego_end_frame_data.empty:
                scenario_code= 'None'
            else:
                init_relative_x, init_relative_y, dhw1, v_x1, v_y1, acc_x1, acc_y1 = parameter_calculate(ego_init_frame_data, BV_init_frame_data)
                end_relative_x, end_relative_y, dhw2, v_x2, v_y2, acc_x2, acc_y2 = parameter_calculate(ego_end_frame_data, BV_end_frame_data)
                if init_relative_x==00 or end_relative_x==00:
                    scenario_code= 'None'
                else:
                    pass

                delta_relative_x=end_relative_x-init_relative_x
                delta_relative_y=end_relative_y-init_relative_y

                # print(BV_LC_inti_flag, BV_LC_end_flag, min_acc_x_BV, BV_lane_change_flag)
                if len(BV_lane_change_flag) == 1: # 无换道
                    if (init_relative_x>0) and (end_relative_x>0) and (delta_relative_x>1) :  # 减速度阈值设为-0.75，可以根据实际情况调整
                        scenario_code=1
                    elif (init_relative_x>0) and (end_relative_x>0) and (delta_relative_x<-1):
                        scenario_code=2
                    elif (init_relative_x>0) and (end_relative_x>0) and (delta_relative_x>-1) and (delta_relative_x<1):
                        scenario_code=0
                    elif (init_relative_x>0) and (end_relative_x<0) and (delta_relative_x<-1):
                        scenario_code=2
                    elif (init_relative_x>0) and (end_relative_x<0) and (delta_relative_x>-1):
                        scenario_code=0
                    elif (init_relative_x<0) and (end_relative_x<0) and (delta_relative_x>1):
                        scenario_code=2
                    elif (init_relative_x<0) and (end_relative_x<0) and (delta_relative_x<-1):
                        scenario_code=1
                    elif (init_relative_x<0) and (end_relative_x<0) and (delta_relative_x>-1) and (delta_relative_x<1):
                        scenario_code=0
                    elif (init_relative_x<0) and (end_relative_x>0) and (delta_relative_x>1):
                        scenario_code=1
                    elif (init_relative_x<0) and (end_relative_x>0) and (delta_relative_x<1):
                        scenario_code=0
                else: # 发生了换道
                    if BV_local_flag=='precedingId': #本车道
                        if delta_relative_x>1:
                            scenario_code=3
                        elif (delta_relative_x<1) and (delta_relative_x>-1):
                            scenario_code=4
                        elif delta_relative_x<-1:
                            scenario_code=5
                            
                    else: # 临车道
                        if 'left' in BV_local_flag:
                            if ego_laneId in BV_lane_change_flag: # 发生了本车道的车道变道
                                if delta_relative_x>1:
                                    scenario_code=6
                                elif delta_relative_x<-1:
                                    scenario_code=7
                                elif (delta_relative_x<1) and (delta_relative_x>-1):
                                    scenario_code=8
                            else: # 发生了其他车道的车道变道
                                if delta_relative_x>1:
                                    scenario_code=3
                                elif delta_relative_x<-1:
                                    scenario_code=4
                                elif (delta_relative_x<1) and (delta_relative_x>-1):
                                    scenario_code=5
                            
                        elif 'right' in BV_local_flag:
                            if ego_laneId in BV_lane_change_flag: # 发生了本车道的车道变道
                                if delta_relative_x>1:
                                    scenario_code=6
                                elif delta_relative_x<-1:
                                    scenario_code=7
                                elif (delta_relative_x<1) and (delta_relative_x>-1):
                                    scenario_code=8
                            else: # 发生了其他车道的车道变道
                                if delta_relative_x>1:
                                    scenario_code=3
                                elif delta_relative_x<-1:
                                    scenario_code=4
                                elif (delta_relative_x<1) and (delta_relative_x>-1):
                                    scenario_code=5
    # print("scenario_code",scenario_code)
    return scenario_code

def parameter_calculate(ego_frame_data, BV_frame_data):
    x1=ego_frame_data['x'].values[0]
    y1=ego_frame_data['y'].values[0]
    ego_offset=ego_frame_data['ego_offset'].values[0]
    ego_lane_id=ego_frame_data['laneId'].values[0]
    ego_width=ego_frame_data['width'].values[0]

    x_obj = BV_frame_data['x'].values[0]
    y_obj = BV_frame_data['y'].values[0]
    v_x = BV_frame_data['v_x_lane'].values[0]
    v_y = BV_frame_data['v_y_lane'].values[0]
    acc_x = BV_frame_data['acc_x_lane'].values[0]
    acc_y = BV_frame_data['acc_y_lane'].values[0]
    obj_offset = BV_frame_data['ego_offset'].values[0]
    obj_lane_id = BV_frame_data['laneId'].values[0]
    obj_width = BV_frame_data['width'].values[0]

    if ego_lane_id ==obj_lane_id:
        y_relative=0
        x_relative=np.sqrt((x1-x_obj)**2+(y1-y_obj)**2-y_relative**2)

    elif ego_lane_id>obj_lane_id:
        y_relative=3.75
        x_relative=np.sqrt((x1-x_obj)**2+(y1-y_obj)**2-y_relative**2)

    elif ego_lane_id<obj_lane_id:
        y_relative=-3.75
        x_relative=np.sqrt((x1-x_obj)**2+(y1-y_obj)**2-y_relative**2)
    
    if (x1-x_obj)**2+(y1-y_obj)**2<y_relative**2:
        x_relative=00
    else:
        pass
    
    if x1<x_obj:
        x_relative=x_relative
        dhw=x_relative-(0.5*ego_width+0.5*obj_width)
    else:
        x_relative=-x_relative
        dhw=x_relative+(0.5*ego_width+0.5*obj_width)

    return x_relative, y_relative,dhw, v_x, v_y, acc_x, acc_y

def parameter_extract(df, init_frame, ego_frame_data, sce_type):
    BV_frame_data1=pd.DataFrame()
    if sce_type =='FD-FD' or sce_type=='FD-LC':
        BV_id_p=ego_frame_data['precedingId'].values[0]
        BV_frame_data1=df.loc[(df['ego_id']==BV_id_p) & (df['frame']==init_frame)]
    else:
        pass
    
    if BV_frame_data1.empty:
        BV_id_p='None'
        x_relative1=0
        y_relative1=0
        dhw1=0
        v_x1=0
        v_y1=0
        acc_x1=0
        acc_y1=0
    else:
        x_relative1, y_relative1,dhw1, v_x1, v_y1, acc_x1, acc_y1=parameter_calculate(ego_frame_data, BV_frame_data1)
    
    return BV_id_p, x_relative1, y_relative1,dhw1, v_x1, v_y1, acc_x1, acc_y1

source_directory = './output'

# for q in range(1,69):
source = source_directory

# trackdata = data_adjustment(source,k)
trackdata = source+'/Round_Scenario_1.csv'
file_name = source+'/scenario_code_1.csv'

header = ['inti_frame','end_frame','ego_id','x','y','ego_offset','width','height','v_x_lane','v_y_lane','acc_x_lane','acc_y_lane','dhw','thw','ttc','laneId','precedingId','followingId','leftPrecedingId',  'leftAlongsideId',  'leftFollowingId',  'rightPrecedingId', 'rightAlongsideId', 'rightFollowingId','orientation','yaw_rate','Scenario_type','BV_id_1','x_relative_1','y_relative_1','dhw_1','v_x_1','v_y_1','acc_x_1','acc_y_1','BV_Behav_type_1','BV_Behav_type_2','BV_Behav_type_3','BV_Behav_type_4','BV_Behav_type_5','BV_Behav_type_6','BV_Behav_type_7','code']#frame,i,x,y,ego_offset,ego_width,ego_height,v_x_lane,v_y_lane,acc_x_lane,acc_y_lane,dhw,thw,ttc,ego_lane_id,preceding_id,following_id,left_preceding_id,left_alongside_id,left_following_id,right_preceding_id,right_alongside_id,right_following_id,orientation,yaw_rate

with open(file_name, 'w', newline='') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerow(header)    

# Read the files
df2= pd.read_csv(trackdata)

#result_frame=pd.DataFrame(columns=['id', 'ego_init_frame', 'ego_end_frame','Scenario_type'])

# 根据id和numLaneChanges进行筛选，筛选出没有发生车道变道的id
# print(id_range)
            
# 在“tracksdata”中找到该id的场景起始帧和结束帧
ego_track=df2[df2['ego_id']==15]
ego_frame_range_actual=ego_track['frame'].unique()
ego_end_frame=ego_frame_range_actual[-1]
ego_init_frame=ego_frame_range_actual[0]
ego_lane_change_flag=ego_track['laneId'].unique()
BV_LC_flag=ego_track['precedingId'].unique()
BV_LC_id=0

# 根据ego car的减速度和换道情况，确定场景的初始帧
if len(ego_lane_change_flag) == 1: # 无换道
    if len(BV_LC_flag)==1:
        if BV_LC_flag[0] == 100:
            scenario_type='False'
        else:
            scenario_type='FD-FD'
    elif len(BV_LC_flag)==2:
        if 100 in BV_LC_flag:
            for id in BV_LC_flag:
                if id == 100:
                    pass
                else:
                    BV_lane_change_flag=df2[df2['ego_id']==id]['laneId'].unique()
                    if len(BV_lane_change_flag) == 1:
                        scenario_type='FD-FD'
                    else:
                        scenario_type='LIN-FD'
        else:
            scenario_type='LIN-FD'                
    ego_intention=1

else: # 发生了换道
    scenario_type='FD-LC'  
    ego_intention=2
# print('ego_init_frame:', ego_init_frame, 'ego_end_frame:', ego_end_frame)
# print(ego_track_data)

eifd=ego_track[ego_track['frame']==ego_init_frame] # 找到ego car的初始帧数据

BV_id1, x_relative1, y_relative1,dhw1, v_x1, v_y1, acc_x1, acc_y1=parameter_extract(df2, ego_init_frame, eifd, scenario_type)

result_frame=[eifd['frame'].values[0], ego_end_frame, eifd['ego_id'].values[0], eifd['x'].values[0], eifd['y'].values[0], eifd['ego_offset'].values[0], eifd['width'].values[0], eifd['height'].values[0], eifd['v_x_lane'].values[0], eifd['v_y_lane'].values[0], eifd['acc_x_lane'].values[0], eifd['acc_y_lane'].values[0], eifd['dhw'].values[0], eifd['thw'].values[0], eifd['ttc'].values[0], eifd['laneId'].values[0], eifd['precedingId'].values[0], eifd['followingId'].values[0], eifd['leftPrecedingId'].values[0], eifd['leftAlongsideId'].values[0], eifd['leftFollowingId'].values[0], eifd['rightPrecedingId'].values[0], eifd['rightAlongsideId'].values[0], eifd['rightFollowingId'].values[0], eifd['orientation'].values[0], eifd['yaw_rate'].values[0], scenario_type, BV_id1, x_relative1, y_relative1,dhw1, v_x1, v_y1, acc_x1, acc_y1]
m=0
for col in ['precedingId', 'leftPrecedingId', 'leftAlongsideId', 'leftFollowingId', 'rightPrecedingId', 'rightAlongsideId', 'rightFollowingId']:
    m+=1
    BV_id=eifd[col].values[0]
    BV_local_flag=col
    ego_laneID=eifd['laneId'].values
    # print('BV_id:', BV_id)
    if BV_id == 0:
        BV_id_list=ego_track.loc[(ego_track['frame']<=ego_end_frame) & (ego_track['frame']>=ego_init_frame)][col]
        # print(BV_id_list)
        BV_id=next((x for x in BV_id_list if x != 0), 0)
    else:
        pass
    # print('BV_id:', BV_id)

    if BV_id != 0:
        BV_scenario_code=scenario_code(BV_id, ego_track, ego_laneID, BV_local_flag, ego_end_frame, ego_init_frame, df2)
        result_frame += [str(1)+str(m)+str(BV_scenario_code)]
    else:
        BV_scenario_code=0
        result_frame += [0]
    
    if m==1:
        if BV_scenario_code==0:
            code=None
        else:
            code=str(1)+str(m)+str(BV_scenario_code)
    elif m in [2,3,5,6]:
        # print(m)
        if BV_scenario_code in [3,4,5]:
            code = str(code)+'.'+str(1)+str(m)+str(BV_scenario_code)
            # print(BV_scenario_code, code)
        else:
            pass
    else:
        pass

result_frame+=['1'+str(ego_intention)+'.'+str(code)]

with open(file_name, 'a', newline='') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerow(result_frame)
                  
        
        
        


            
            