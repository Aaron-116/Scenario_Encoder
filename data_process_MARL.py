import pandas as pd
import csv
import os

file_source='./MARL_data/test'

count=0

for filename in os.listdir(file_source):
    count+=1
    file_path=os.path.join(file_source,filename)
    file_name='./output/Round_Scenario_'+str(count)+'.csv'

    header=['frame','ego_id','x','y','ego_offset','width','height','v_x_lane','v_y_lane','acc_x_lane','acc_y_lane','dhw','thw','ttc','laneId','precedingId','followingId','leftPrecedingId',  'leftAlongsideId1', 'leftAlongsideId2', 'leftFollowingId',  'rightPrecedingId', 'rightAlongsideId1', 'rightAlongsideId2','rightFollowingId','orientation','yaw_rate']

    with open(file_name, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(header)

    df=pd.read_csv(file_path)
    ego_track=df[df['idx']==15]
    frame_range=df['step'].unique()
    lane_Id_range=df['laneid'].unique()
    id_range=df['idx'].unique()

    for frame in frame_range:
        ego_frame=ego_track[ego_track['step']==frame]
        ego_lane_Id=ego_frame['laneid'].values[0]
        ego_x=ego_frame['x'].values[0]
        ego_y=ego_frame['y'].values[0]
        ego_v_x=ego_frame['vx'].values[0]
        ego_v_y=ego_frame['vy'].values[0]
        ego_acc_x=ego_frame['ax'].values[0]
        ego_acc_y=ego_frame['ay'].values[0]
        ego_ttc=ego_frame['TTC'].values[0]
        ego_lane_Id=ego_frame['laneid'].values[0]
        allframe_data=df[df['step']==frame]
        # print(ego_lane_Id)
        BV_egolane_frame=allframe_data[allframe_data['laneid']==ego_lane_Id]
        # print(len(BV_egolane_frame))
        if BV_egolane_frame.empty:
            proceding_BV_id_only=100
        else:
            proceding_BV_id=pd.DataFrame(columns=['BV_id','x_relative'])
            for x in BV_egolane_frame['x']:
                x_relative=x-ego_x
                # print(x,x_relative)
                if x_relative>0:
                    BV_id=BV_egolane_frame[BV_egolane_frame['x']==x]['idx'].values[0]
                    result=[BV_id,x_relative]
                    # print(result)
                    proceding_BV_id.loc[len(proceding_BV_id)]=result
                else:
                    pass
            # print("前方：",len(proceding_BV_id))
            if len(proceding_BV_id)>1:
                min_index=proceding_BV_id['x_relative'].idxmin()
                proceding_BV_id_only=proceding_BV_id.loc[min_index,'BV_id']
            elif len(proceding_BV_id)==0:
                proceding_BV_id_only=100
            else:
                proceding_BV_id_only=proceding_BV_id['BV_id'].values[0]

        if (ego_lane_Id-1) in lane_Id_range:
            BV_rightlane_frame=allframe_data[allframe_data['laneid']==ego_lane_Id-1]
            # print("右侧：",len(BV_rightlane_frame))
            if BV_rightlane_frame.empty:
                BV_rightlane_proceding_id_only=100
                BV_rightlane_f_id_only=100
                BV_rightlane_a_id1=100
                BV_rightlane_a_id2=100
            else:
                BV_rightlane_id=pd.DataFrame(columns=['BV_id','x_relative','position'])
                for BV_id in BV_rightlane_frame['idx']:
                    BV_x=BV_rightlane_frame[BV_rightlane_frame['idx']==BV_id]['x'].values[0]
                    BV_y=BV_rightlane_frame[BV_rightlane_frame['idx']==BV_id]['y'].values[0]
                    x_relative=(((BV_x-ego_x)**2+(BV_y-ego_y)**2)-3.5**2)**0.5
                    if x_relative>5 and BV_x>ego_x:
                        BV_position=5
                    elif x_relative>5 and BV_x<ego_x:
                        BV_position=7
                    else:
                        BV_position=6
                    
                    result=[BV_id,x_relative,BV_position]
                    BV_rightlane_id.loc[len(BV_rightlane_id)]=result
                # print("right:",len(BV_rightlane_id))
                BV_rightlane_proceding_id=BV_rightlane_id[BV_rightlane_id['position']==5]
                if len(BV_rightlane_proceding_id)>1:
                    min_index=BV_rightlane_proceding_id['x_relative'].idxmin()
                    BV_rightlane_proceding_id_only=BV_rightlane_proceding_id.loc[min_index,'BV_id']
                elif len(BV_rightlane_proceding_id)==0:
                    BV_rightlane_proceding_id_only=100
                else:
                    BV_rightlane_proceding_id_only=BV_rightlane_proceding_id['BV_id'].values[0]

                BV_rightlane_f_id=BV_rightlane_id[BV_rightlane_id['position']==7]
                if len(BV_rightlane_f_id)>1:
                    min_index=BV_rightlane_f_id['x_relative'].idxmin()
                    BV_rightlane_f_id_only=BV_rightlane_f_id.loc[min_index,'BV_id']
                elif len(BV_rightlane_f_id)==0:
                    BV_rightlane_f_id_only=100
                else:
                    BV_rightlane_f_id_only=BV_rightlane_f_id['BV_id'].values[0]

                BV_rightlane_a_id=BV_rightlane_id[BV_rightlane_id['position']==6]
                if len(BV_rightlane_a_id)>1:
                    BV_rightlane_a_id1=BV_rightlane_a_id['BV_id'].values[0]
                    BV_rightlane_a_id2=BV_rightlane_a_id['BV_id'].values[1]
                elif len(BV_rightlane_a_id)==0:
                    BV_rightlane_a_id1=100
                    BV_rightlane_a_id2=100
                else:
                    BV_rightlane_a_id1=BV_rightlane_a_id['BV_id'].values[0]
                    BV_rightlane_a_id2=100

        else:
            BV_rightlane_proceding_id_only=100
            BV_rightlane_f_id_only=100
            BV_rightlane_a_id1=100
            BV_rightlane_a_id2=100

        if (ego_lane_Id+1) in lane_Id_range:
            BV_leftlane_frame=allframe_data[allframe_data['laneid']==ego_lane_Id+1]
            # print("左侧：",len(BV_leftlane_frame))
            if BV_leftlane_frame.empty:
                BV_leftlane_p_id_only=100
                BV_leftlane_f_id_only=100
                BV_leftlane_a_id1=100
                BV_leftlane_a_id2=100
            else:
                BV_leftlane_id=pd.DataFrame(columns=['BV_id','x_relative','position'])
                for BV_id in BV_leftlane_frame['idx']:
                    BV_x=BV_leftlane_frame[BV_leftlane_frame['idx']==BV_id]['x'].values[0]
                    BV_y=BV_leftlane_frame[BV_leftlane_frame['idx']==BV_id]['y'].values[0]
                    x_relative=(((BV_x-ego_x)**2+(BV_y-ego_y)**2)-3.5**2)**0.5
                    if x_relative>5 and BV_x>ego_x:
                        BV_position=2
                    elif x_relative>5 and BV_x<ego_x:
                        BV_position=4
                    else:
                        BV_position=3
                    result=[BV_id,x_relative,BV_position]
                    # print(result)
                    BV_leftlane_id.loc[len(BV_leftlane_id)]=result
                    
                # print("left:",len(BV_leftlane_id))
                BV_leftlane_p_id=BV_leftlane_id[BV_leftlane_id['position']==2]
                if len(BV_leftlane_p_id)>1:
                    min_index=BV_leftlane_p_id['x_relative'].idxmin()
                    BV_leftlane_p_id_only=BV_leftlane_p_id.loc[min_index,'BV_id']
                elif len(BV_leftlane_p_id)==0:
                    BV_leftlane_p_id_only=100
                else:
                    BV_leftlane_p_id_only=BV_leftlane_p_id['BV_id'].values[0]

                BV_leftlane_f_id=BV_leftlane_id[BV_leftlane_id['position']==4]
                if len(BV_leftlane_f_id)>1:  
                    min_index=BV_leftlane_f_id['x_relative'].idxmin()
                    BV_leftlane_f_id_only=BV_leftlane_f_id.loc[min_index,'BV_id']
                elif len(BV_leftlane_f_id)==0:
                    BV_leftlane_f_id_only=100
                else:
                    BV_leftlane_f_id_only=BV_leftlane_f_id['BV_id'].values[0]

                BV_leftlane_a_id=BV_leftlane_id[BV_leftlane_id['position']==3]
                if len(BV_leftlane_a_id)>1:
                    BV_leftlane_a_id1=BV_leftlane_a_id['BV_id'].values[0]
                    BV_leftlane_a_id2=BV_leftlane_a_id['BV_id'].values[1]
                elif len(BV_leftlane_a_id)==0:
                    BV_leftlane_a_id1=100
                    BV_leftlane_a_id2=100
                else:
                    BV_leftlane_a_id1=BV_leftlane_a_id['BV_id'].values[0]
                    BV_leftlane_a_id2=100
        else:
            BV_leftlane_p_id_only=100
            BV_leftlane_f_id_only=100
            BV_leftlane_a_id1=100
            BV_leftlane_a_id2=100
        
        result=[frame,15,ego_x,ego_y,0,0,0,ego_v_x,ego_v_y,ego_acc_x,ego_acc_y,0,0,ego_ttc,ego_lane_Id,proceding_BV_id_only,0,BV_leftlane_p_id_only,BV_leftlane_a_id1,BV_leftlane_a_id2,BV_leftlane_f_id_only,BV_rightlane_proceding_id_only,BV_rightlane_a_id1,BV_rightlane_a_id2,BV_rightlane_f_id_only,0,0]

        with open(file_name, 'a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(result)

    for id in id_range:
        if id != 15:
            BV_data=df[df['idx']==id]
            f_range=BV_data['step'].unique()
            for frame in f_range:
                BV_frame=BV_data[BV_data['step']==frame]
                BV_x=BV_frame['x'].values[0]
                BV_y=BV_frame['y'].values[0]
                BV_v_x=BV_frame['vx'].values[0]
                BV_v_y=BV_frame['vy'].values[0]
                BV_acc_x=BV_frame['ax'].values[0]
                BV_acc_y=BV_frame['ay'].values[0]
                BV_ttc=BV_frame['TTC'].values[0]
                BV_lane_Id=BV_frame['laneid'].values[0]

                result=[frame,id,BV_x,BV_y,0,0,0,BV_v_x,BV_v_y,BV_acc_x,BV_acc_y,0,0,BV_ttc,BV_lane_Id,0,0,0,0,0,0,0,0,0,0,0,0]
                with open(file_name, 'a', newline='') as csv_file:
                    writer = csv.writer(csv_file)
                    writer.writerow(result)
        else:
            pass


    
