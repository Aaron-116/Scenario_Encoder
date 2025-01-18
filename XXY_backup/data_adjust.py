import pandas as pd
import numpy as np
import csv
import os

def data_adjustment(source, k):
    for filename in os.listdir(source):
        # print(filename)
        if filename.endswith('tracks.csv'):
            file_path = os.path.join(source, filename)
            df=pd.read_csv(file_path)

            # 确认“id”字段的取值范围
            id_range = df['id'].unique()

            header = ['frame','ego_id','x','y','ego_offset','width','height','v_x_lane','v_y_lane','acc_x_lane','acc_y_lane','dhw','thw','ttc','laneId','precedingId','followingId','leftPrecedingId',  'leftAlongsideId',  'leftFollowingId',  'rightPrecedingId', 'rightAlongsideId', 'rightFollowingId','orientation','yaw_rate']
            #frame,i,x,y,ego_offset,ego_width,ego_height,v_x_lane,v_y_lane,acc_x_lane,acc_y_lane,dhw,thw,ttc,ego_lane_id,preceding_id,following_id,left_preceding_id,left_alongside_id,left_following_id,right_preceding_id,right_alongside_id,right_following_id,orientation,yaw_rate
            
            file_name = source+'/tracks_rejust_10fps.csv'

            with open(file_name, 'w', newline='') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(header)
            
            count1 = 0
            count2 = 0

            # 优化遍历过程，减少不必要的重复计算
            for i in id_range:
                count1 += 1
                id_data = df[df['id'] == i]
                frame_range = id_data['frame'].unique()
                under_frame_range = [frame for frame in frame_range if frame % 3 == 0]

                for frame in under_frame_range:
                    count2 += 1
                    frame_data = id_data[id_data['frame'] == frame]  # 只取第一行数据
                    # print(frame_data)
                    x=frame_data['x'].values[0]
                    y=frame_data['y'].values[0]
                    ego_offset=frame_data['ego_offset'].values[0]
                    ego_lane_id=frame_data['laneId'].values[0]
                    ego_width=frame_data['width'].values[0]
                    ego_height=frame_data['height'].values[0]
                    ttc = frame_data['ttc'].values[0]
                    dhw = frame_data['dhw'].values[0]
                    thw = frame_data['thw'].values[0]
                    v_x = frame_data['xVelocity'].values[0]
                    v_y = frame_data['yVelocity'].values[0]
                    acc_x = frame_data['xAcceleration'].values[0]
                    acc_y = frame_data['yAcceleration'].values[0]
                    frontsidedistance=frame_data['frontSightDistance'].values[0]
                    backsidedistance=frame_data['backSightDistance'].values[0]
                    angle = frame_data['angle'].values[0]
                    yaw_rate = frame_data['yaw_rate '].values[0]
                    orientation = frame_data['orientation'].values[0]
                    preceding_id = frame_data['precedingId'].values[0]
                    following_id = frame_data['followingId'].values[0]
                    left_preceding_id = frame_data['leftPrecedingId'].values[0]
                    left_alongside_id = frame_data['leftAlongsideId'].values[0]
                    left_following_id = frame_data['leftFollowingId'].values[0]
                    right_preceding_id = frame_data['rightPrecedingId'].values[0]
                    right_alongside_id = frame_data['rightAlongsideId'].values[0]
                    right_following_id = frame_data['rightFollowingId'].values[0]

                    if v_x == 0:
                        continue
                    elif v_x>0:        
                        v_x_lane = v_x * np.cos(np.radians(angle)) + v_y * np.sin(np.radians(angle))
                        v_y_lane = -v_x * np.sin(np.radians(angle)) + v_y * np.cos(np.radians(angle))
                        acc_x_lane = acc_x * np.cos(np.radians(angle)) + acc_y * np.sin(np.radians(angle))
                        acc_y_lane = -acc_x * np.sin(np.radians(angle)) + acc_y * np.cos(np.radians(angle))
                    else:
                        v_x_lane = -v_x * np.cos(np.radians(angle)) + v_y * np.sin(np.radians(angle))
                        v_y_lane = v_x * np.sin(np.radians(angle)) + v_y * np.cos(np.radians(angle))
                        acc_x_lane = -acc_x * np.cos(np.radians(angle)) + acc_y * np.sin(np.radians(angle))
                        acc_y_lane = acc_x * np.sin(np.radians(angle)) + acc_y * np.cos(np.radians(angle))
                        x=backsidedistance

                    # print("corresponding_data:", corresponding_data)
                    result_values = [frame,i,x,y,ego_offset,ego_width,ego_height,v_x_lane,v_y_lane,acc_x_lane,acc_y_lane,dhw,thw,ttc,ego_lane_id,preceding_id,following_id,left_preceding_id,left_alongside_id,left_following_id,right_preceding_id,right_alongside_id,right_following_id,orientation,yaw_rate]

                    with open(file_name, 'a', newline='') as csv_file:
                        writer = csv.writer(csv_file)
                        writer.writerow(result_values)

                print("处理文件夹：",'/DJI_00'+str(k).zfill(2),"轨迹总数：", len(id_range), "已处理轨迹数：", count1)
    return file_name