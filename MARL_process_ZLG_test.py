import pandas as pd
import csv
import os

# 每条road的长度
map = {
    2: 350,
    16: 445,
    13: 1798,
    3: 132,
    22: 427
}


def get_s(roadid, roads):
    """
    计算将相对s转换为绝对s。
    """
    if roadid == 2:
        s = roads
    elif roadid == 16:
        s = map[2] + roads
    elif roadid == 13:
        s = map[2] + map[16] + roads
    else:
        s = -1000
    return s


def process(file_source, out_path, ego_id, max_count):
    count = 0
    for filename in os.listdir(file_source):
        name = filename[:-4]
        count += 1
        file_path = os.path.join(file_source, filename)
        file_name = out_path + '/' + 'Round_Scenario_' + str(count) + '.csv'
        # 输出的header
        header = ['frame', 'id', 'x', 'y', 's', 'v_x_lane', 'v_y_lane', 'acc_x_lane', 'acc_y_lane', 'dhw', 'thw', 'ttc',
                  'laneId',
                  'BV_1', 'BV_2', 'BV_3', 'BV_4', 'BV_5', 'BV_6', 'BV_7', 'BV_8', 'BV_0',
                  's1', 's2', 's3', 's4', 's5', 's6', 's7', 's8', 's0',
                  'file_name']
        with open(file_name, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(header)

        df = pd.read_csv(file_path)
        df['roadS'] = df.apply(lambda row: get_s(row['roadid'], row['roadS']), axis=1)
        ego_track = df[df['idx'] == ego_id]
        frame_range = df['step'].unique()
        lane_Id_range = df['laneid'].unique()
        id_range = df['idx'].unique()

        for frame in frame_range:
            ego_frame = ego_track[ego_track['step'] == frame]
            ego_lane_Id = ego_frame['laneid'].values[0]
            ego_x = ego_frame['x'].values[0]
            ego_y = ego_frame['y'].values[0]
            ego_s = ego_frame['roadS'].values[0]
            ego_v_x = ego_frame['vx'].values[0]
            ego_v_y = ego_frame['vy'].values[0]
            ego_acc_x = ego_frame['ax'].values[0]
            ego_acc_y = ego_frame['ay'].values[0]
            ego_ttc = ego_frame['TTC'].values[0]
            allframe_data = df[df['step'] == frame]
            BV_egolane_frame = allframe_data[allframe_data['laneid'] == ego_lane_Id]
            BV_egolane_frame = BV_egolane_frame[BV_egolane_frame['idx'] != ego_id]

            BV = {
                1: -1,
                2: -1,
                3: -1,
                4: -1,
                5: -1,
                6: -1,
                7: -1,
                8: -1,
                0: -1
            }

            s_rela = {
                1: None,
                2: None,
                3: None,
                4: None,
                5: None,
                6: None,
                7: None,
                8: None,
                0: None
            }

            '''筛选位于 Segment 2 7 的BV'''
            if BV_egolane_frame.empty:
                pass
            else:
                BV_front_all = pd.DataFrame(columns=['BV_id', 's_relative', 'position'])
                BV_back_all = pd.DataFrame(columns=['BV_id', 's_relative', 'position'])
                for s in BV_egolane_frame['roadS']:
                    s_relative = s - ego_s
                    if 5 < s_relative < 105:
                        BV_id = BV_egolane_frame[BV_egolane_frame['roadS'] == s]['idx'].values[0]
                        position = 2
                        result = [BV_id, s_relative, position]
                        BV_front_all.loc[len(BV_front_all)] = result
                    elif -55 < s_relative < -5:
                        BV_id = BV_egolane_frame[BV_egolane_frame['roadS'] == s]['idx'].values[0]
                        position = 7
                        result = [BV_id, s_relative, position]
                        BV_back_all.loc[len(BV_back_all)] = result
                    elif -5 <= s_relative <= 5:
                        BV_id = BV_egolane_frame[BV_egolane_frame['roadS'] == s]['idx'].values[0]
                        BV[0] = BV_id
                        s_rela[0] = 0
                    else:
                        pass

                if len(BV_front_all) > 1:
                    min_index = BV_front_all['s_relative'].idxmin()
                    BV[2] = BV_front_all.loc[min_index, 'BV_id']
                    s_rela[2] = BV_front_all.loc[min_index, 's_relative']
                elif len(BV_front_all) == 0:
                    pass
                else:
                    BV[2] = BV_front_all['BV_id'].values[0]
                    s_rela[2] = BV_front_all['s_relative'].values[0]

                if len(BV_back_all) > 1:
                    max_index = BV_back_all['s_relative'].idxmax()
                    BV[7] = BV_back_all.loc[max_index, 'BV_id']
                    s_rela[7] = BV_back_all.loc[max_index, 's_relative']
                elif len(BV_back_all) == 0:
                    pass
                else:
                    BV[7] = BV_back_all['BV_id'].values[0]
                    s_rela[7] = BV_back_all['s_relative'].values[0]

            '''筛选位于 Segment 3 5 8 (右侧)的BV'''
            if (ego_lane_Id - 1) in lane_Id_range:
                BV_right_all = allframe_data[allframe_data['laneid'] == ego_lane_Id - 1]
                if BV_right_all.empty:
                    pass
                else:
                    BV_right_filtered = pd.DataFrame(columns=['BV_id', 's_relative', 'position'])
                    for BV_id in BV_right_all['idx']:
                        BV_s = BV_right_all[BV_right_all['idx'] == BV_id]['roadS'].values[0]
                        s_relative = BV_s - ego_s
                        if -5 < s_relative < 5:
                            BV_position = 5
                        elif -55 < s_relative <= -5:
                            BV_position = 8
                        elif 5 <= s_relative <= 105:
                            BV_position = 3
                        else:
                            BV_position = 0
                        result = [BV_id, s_relative, BV_position]
                        BV_right_filtered.loc[len(BV_right_filtered)] = result

                    BV_5_all = BV_right_filtered[BV_right_filtered['position'] == 5]
                    if len(BV_5_all) > 1:
                        min_index = BV_5_all['s_relative'].idxmin()
                        BV[5] = BV_5_all.loc[min_index, 'BV_id']
                        s_rela[5] = BV_5_all.loc[min_index, 's_relative']
                    elif len(BV_5_all) == 0:
                        pass
                    else:
                        BV[5] = BV_5_all['BV_id'].values[0]
                        s_rela[5] = BV_5_all['s_relative'].values[0]

                    BV_3_all = BV_right_filtered[BV_right_filtered['position'] == 3]
                    if len(BV_3_all) > 1:
                        min_index = BV_3_all['s_relative'].idxmin()
                        BV[3] = BV_3_all.loc[min_index, 'BV_id']
                        s_rela[3] = BV_3_all.loc[min_index, 's_relative']
                    elif len(BV_3_all) == 0:
                        pass
                    else:
                        BV[3] = BV_3_all['BV_id'].values[0]
                        s_rela[3] = BV_3_all['s_relative'].values[0]

                    BV_8_all = BV_right_filtered[BV_right_filtered['position'] == 8]
                    if len(BV_8_all) > 1:
                        max_index = BV_8_all['s_relative'].idxmax()
                        BV[8] = BV_8_all.loc[max_index, 'BV_id']
                        s_rela[8] = BV_8_all.loc[max_index, 's_relative']
                    elif len(BV_8_all) == 0:
                        pass
                    else:
                        BV[8] = BV_8_all['BV_id'].values[0]
                        s_rela[8] = BV_8_all['s_relative'].values[0]

            '''筛选位于 Segment 1 4 6 (左侧)的BV'''
            if (ego_lane_Id + 1) in lane_Id_range:
                BV_left_all = allframe_data[allframe_data['laneid'] == ego_lane_Id + 1]
                if BV_left_all.empty:
                    pass
                else:
                    BV_left_filtered = pd.DataFrame(columns=['BV_id', 's_relative', 'position'])
                    for BV_id in BV_left_all['idx']:
                        BV_s = BV_left_all[BV_left_all['idx'] == BV_id]['roadS'].values[0]
                        s_relative = BV_s - ego_s
                        if -5 < s_relative < 5:
                            BV_position = 4
                        elif 5 <= s_relative < 105:
                            BV_position = 1
                        elif -55 < s_relative <= -5:
                            BV_position = 6
                        else:
                            BV_position = 0
                        result = [BV_id, s_relative, BV_position]
                        BV_left_filtered.loc[len(BV_left_filtered)] = result

                    BV_4_all = BV_left_filtered[BV_left_filtered['position'] == 4]
                    if len(BV_4_all) > 1:
                        min_index = BV_4_all['s_relative'].idxmin()
                        BV[4] = BV_4_all.loc[min_index, 'BV_id']
                        s_rela[4] = BV_4_all.loc[min_index, 's_relative']
                    elif len(BV_4_all) == 0:
                        pass
                    else:
                        BV[4] = BV_4_all['BV_id'].values[0]
                        s_rela[4] = BV_4_all['s_relative'].values[0]

                    BV_1_all = BV_left_filtered[BV_left_filtered['position'] == 1]
                    if len(BV_1_all) > 1:
                        min_index = BV_1_all['s_relative'].idxmin()
                        BV[1] = BV_1_all.loc[min_index, 'BV_id']
                        s_rela[1] = BV_1_all.loc[min_index, 's_relative']
                    elif len(BV_1_all) == 0:
                        pass
                    else:
                        BV[1] = BV_1_all['BV_id'].values[0]
                        s_rela[1] = BV_1_all['s_relative'].values[0]

                    BV_6_all = BV_left_filtered[BV_left_filtered['position'] == 6]
                    if len(BV_6_all) > 1:
                        max_index = BV_6_all['s_relative'].idxmax()
                        BV[6] = BV_6_all.loc[max_index, 'BV_id']
                        s_rela[6] = BV_6_all.loc[max_index, 's_relative']
                    elif len(BV_6_all) == 0:
                        pass
                    else:
                        BV[6] = BV_6_all['BV_id'].values[0]
                        s_rela[6] = BV_6_all['s_relative'].values[0]

            # header = ['frame', 'ego_id', 'x', 'y', 's', 'v_x_lane', 'v_y_lane', 'acc_x_lane', 'acc_y_lane', 'dhw', 'thw',
            #           'ttc', 'laneId', 'BV_1', 'BV_2', 'BV_3', 'BV_4', 'BV_5', 'BV_6', 'BV_7', 'BV_8']
            result = [frame, ego_id, ego_x, ego_y, ego_s, ego_v_x, ego_v_y, ego_acc_x, ego_acc_y, None, None, ego_ttc,
                      ego_lane_Id,
                      BV[1], BV[2], BV[3], BV[4], BV[5], BV[6], BV[7], BV[8], BV[0],
                      s_rela[1], s_rela[2], s_rela[3], s_rela[4], s_rela[5], s_rela[6], s_rela[7], s_rela[8], s_rela[0],
                      name]
            with open(file_name, 'a', newline='') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(result)

        for id in id_range:
            if id != ego_id:
                BV_data = df[df['idx'] == id]
                f_range = BV_data['step'].unique()
                for frame in f_range:
                    BV_frame = BV_data[BV_data['step'] == frame]
                    BV_x = BV_frame['x'].values[0]
                    BV_y = BV_frame['y'].values[0]
                    BV_s = BV_frame['roadS'].values[0]
                    BV_v_x = BV_frame['vx'].values[0]
                    BV_v_y = BV_frame['vy'].values[0]
                    BV_acc_x = BV_frame['ax'].values[0]
                    BV_acc_y = BV_frame['ay'].values[0]
                    BV_ttc = BV_frame['TTC'].values[0]
                    BV_lane_Id = BV_frame['laneid'].values[0]
                    # header = ['frame', 'ego_id', 'x', 'y', 's', 'v_x_lane', 'v_y_lane', 'acc_x_lane', 'acc_y_lane', 'dhw',
                    #           'thw', 'ttc', 'laneId', 'BV_1', 'BV_2', 'BV_3', 'BV_4', 'BV_5', 'BV_6', 'BV_7', 'BV_8']
                    result = [frame, id, BV_x, BV_y, BV_s, BV_v_x, BV_v_y, BV_acc_x, BV_acc_y, None, None, BV_ttc,
                              BV_lane_Id,
                              None, None, None, None, None, None, None, None, None,
                              None, None, None, None, None, None, None, None,
                              None]
                    with open(file_name, 'a', newline='') as csv_file:
                        writer = csv.writer(csv_file)
                        writer.writerow(result)
        if count >= max_count:
            break


file_source = './MARL_data/test'
output_path = './output/test/process_test'
ego_id = 15
max_count = 1000
process(file_source, output_path, ego_id, max_count)
