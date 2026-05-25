import math

import numpy as np
import pandas as pd
import csv
import os

ego_id = 20


def get_matrix(ego_track):
    # 定义8个segment的BV_id和s_relative, 分别存储在BV_matrix和s_matrix中, 每个segment最多记录2辆BV
    BV_matrix = -1 * np.ones((2, 9))
    s_matrix = np.nan * np.empty((2, 9))
    # 记录BV的出现顺序
    sequence_matrix = np.zeros((4, 9))
    min_index = ego_track.index[0]
    init_laneid = ego_track.at[min_index, 'laneId']
    # 遍历所有segment
    for i in range(0, 10):
        # 获取当前segment中所有BV的id
        if i == 9:
            m = 0
        else:
            m = i
        data = ego_track[ego_track['laneId'] == init_laneid][f'BV_{m}']
        # 判断出现顺序
        sequence_matrix[0][m - 1] = data[0]
        k = 1
        for j in range(1, len(data)):
            try:
                a = data[j]
            except:
                break
            if data[j] != data[j - 1]:
                sequence_matrix[k][m - 1] = data[j]
                k += 1
            if k > 3:
                break

        # 去除重复的id
        data_unique = data.unique()
        # 过滤掉-1
        data_unique = data_unique[data_unique != -1]
        # 如果没有BV, 则跳过当前segment
        if len(data_unique) == 0:
            continue
        # 如果只有一辆BV, 则将其id存储在BV_matrix中
        if len(data_unique) == 1:
            BV_matrix[0][m - 1] = data_unique[0]
        # 如果有两辆BV, 则将其id存储在BV_matrix中，先出现的会先记录
        elif len(data_unique) == 2:
            BV_matrix[0][m - 1] = data_unique[0]
            BV_matrix[1][m - 1] = data_unique[1]
        # 如果有三辆以上BV, 则跳过当前segment
        else:
            continue

        # 获取当前segment中所有BV的相对位置
        if len(data_unique) == 1:
            data1 = ego_track[f's{m}'].dropna()
            # 获取最小值的索引
            min_index = data1.idxmin()
            start_index = data1.index[0]
            # 获取最小值与初始值的差
            s_relative = data1[min_index] - data1[start_index]
            s_matrix[0][m - 1] = s_relative
        elif len(data_unique) == 2:
            for BV in ego_track[f'BV_{m}']:
                if BV == data_unique[0]:
                    data2 = ego_track[ego_track[f'BV_{m}'] == BV][f's{m}'].dropna()
                elif BV == data_unique[1]:
                    data3 = ego_track[ego_track[f'BV_{m}'] == BV][f's{m}'].dropna()
            min_index2 = data2.idxmin()
            start_index2 = data2.index[0]
            s_relative2 = data2[min_index2] - data2[start_index2]
            min_index3 = data3.idxmin()
            start_index3 = data3.index[0]
            s_relative3 = data3[min_index3] - data3[start_index3]
            s_matrix[0][m - 1] = s_relative2
            s_matrix[1][m - 1] = s_relative3
    return BV_matrix, s_matrix, sequence_matrix


def get_mm_matrix(df, bv_matrix, s_matrix):
    # BV_C BV_A BV_B
    mm_matrix = [
        np.array([-1, -1, -1]),
        np.array(['lane-keep', 'lane-keep', 'lane-keep']),
        np.array(['vel-keep', 'vel-keep', 'vel-keep'])
    ]
    ego_laneId = df[df["id"] == ego_id]['laneId'].unique()
    # 判断自车转向，True: 右转, False: 左转
    turn_flag = ego_laneId[0] > ego_laneId[1]
    if turn_flag:
        m = 2
        n = 7
    else:
        m = 0
        n = 5
    # segment 2
    if bv_matrix[1, 1] != -1:
        mm_matrix[0][0] = bv_matrix[1, 1]
        BV = mm_matrix[0][0]
        laneId = df[df["id"] == BV]['laneId'].unique()
        if len(laneId) == 2:
            bv_turn_flag = laneId[0] > laneId[1]
            # 判断BV转向与自车转向是否一致
            if turn_flag == bv_turn_flag and ego_laneId[0] == laneId[1]:
                mm_matrix[1][0] = 'cut-in'
            elif turn_flag == bv_turn_flag and ego_laneId[1] == laneId[1]:
                mm_matrix[1][0] = 'cut-out'
        if s_matrix[1][1] < -5:
            mm_matrix[2][0] = 'dec'
    else:
        if bv_matrix[0, 1] != -1:
            mm_matrix[0][0] = bv_matrix[0, 1]
            BV = bv_matrix[0][1]
            laneId = df[df["id"] == BV]['laneId'].unique()
            if len(laneId) == 2:
                bv_turn_flag = laneId[0] > laneId[1]
                if turn_flag == bv_turn_flag and ego_laneId[0] == laneId[1]:
                    mm_matrix[1][0] = 'cut-in'
                elif turn_flag == bv_turn_flag and ego_laneId[1] == laneId[1]:
                    mm_matrix[1][0] = 'cut-out'
            if s_matrix[0][1] < -5:
                mm_matrix[2][0] = 'dec'

    # segment 3 or 8
    if bv_matrix[1, m] != -1:
        mm_matrix[0][1] = bv_matrix[1, m]
        BV = mm_matrix[0][1]
        laneId = df[df["id"] == BV]['laneId'].unique()
        if len(laneId) == 2:
            bv_turn_flag = laneId[0] > laneId[1]
            if turn_flag != bv_turn_flag and ego_laneId[0] == laneId[1]:
                mm_matrix[1][1] = 'cut-out'
            elif turn_flag != bv_turn_flag and ego_laneId[1] == laneId[1]:
                mm_matrix[1][1] = 'cut-in'
        if s_matrix[1][m] < -5:
            mm_matrix[2][1] = 'dec'
    else:
        if bv_matrix[0, m] != -1:
            mm_matrix[0][1] = bv_matrix[0, m]
            BV = bv_matrix[0][m]
            laneId = df[df["id"] == BV]['laneId'].unique()
            if len(laneId) == 2:
                bv_turn_flag = laneId[0] > laneId[1]
                if turn_flag != bv_turn_flag and ego_laneId[0] == laneId[1]:
                    mm_matrix[1][1] = 'cut-out'
                elif turn_flag != bv_turn_flag and ego_laneId[1] == laneId[1]:
                    mm_matrix[1][1] = 'cut-in'
            if s_matrix[0][m] < -5:
                mm_matrix[2][1] = 'dec'

    # segment 8 or 6
    if bv_matrix[1, n] != -1:
        mm_matrix[0][2] = bv_matrix[1, n]
        if s_matrix[1][n] > 5:
            mm_matrix[2][2] = 'acc'
    else:
        if bv_matrix[0, n] != -1:
            mm_matrix[0][2] = bv_matrix[0, n]
            if s_matrix[0][n] > 5:
                mm_matrix[2][2] = 'acc'

    # 检查两辆车是否相同
    if mm_matrix[0][0] == mm_matrix[0][1]:
        if mm_matrix[1][0] == 'cut-out' and mm_matrix[1][1] == 'lane-keep':
            mm_matrix[0][1] = -1
        elif mm_matrix[1][1] == 'cut-out' and mm_matrix[1][0] == 'lane-keep':
            mm_matrix[0][0] = -1
    if mm_matrix[0][0] == mm_matrix[0][2] and mm_matrix[0][2] == 'cut-in':
        mm_matrix[0][2] = -1
    return mm_matrix


def encoder_lk(BV_matrix, s_matrix, sequence_matrix):
    code = {
        'maneuver': 0,  # 0: LaneKeep
        'BV_A': 0,  # 0: 不存在BV, 1: 存在BV
        'BV_B': 0,  # 0: 不存在BV, 1: 存在BV
        'BV_C': 0,  # 0: 不存在BV, 1: 存在BV
        'motion_A': 0,  # LaneKeep 0: 纵向保持+横向保持, 1: 纵向接近+横向保持
        'motion_B': 0,  # 0: 纵向保持, 1: 纵向接近
        'motion_C': 0  # LaneKeep 0: 纵向保持+横向保持, 1: 纵向接近+横向保持
    }
    if BV_matrix[0][1] != -1:
        # segment 2 存在BV
        # 获取segment 2 中BV的出现顺序
        sequence_2 = sequence_matrix[:, 1]
        sequence_2 = sequence_2[sequence_2 != 0]
        if BV_matrix[1][1] != -1:
            # segment 2 出现第二辆BV
            # BV_A存在，BV_C切入
            code['BV_A'] = 1
            code['BV_C'] = 1
            if s_matrix[0][1] < -5:
                # 纵向接近+不变道
                code['motion_A'] = 3
            else:
                code['motion_A'] = 0
            if s_matrix[1][1] < -5:
                # 纵向接近+变道
                code['motion_C'] = 4
            else:
                code['motion_C'] = 2
        else:
            # segment 2 只存在一辆BV，判断motion
            if len(sequence_2) == 2 and sequence_2[0] == -1:
                # BV_A不存在，BV_C切入
                code['BV_A'] = 0
                code['BV_C'] = 1
                if s_matrix[1][1] < -5:
                    # 纵向接近+变道
                    code['motion_C'] = 5
                else:
                    # 纵向保持+变道
                    code['motion_C'] = 2
            else:
                # BV_A切出
                code['BV_A'] = 1
                if s_matrix[0][1] < -5:
                    # 纵向接近
                    code['motion_A'] = 3
                else:
                    # 纵向保持或远离
                    pass

    if BV_matrix[0][6] != -1:
        # segment 7 存在BV
        code['BV_B'] = 1
        if BV_matrix[1][6] != -1:
            # segment 7 出现第二辆BV
            if s_matrix[1][6] < -5:
                # 纵向接近
                code['motion_B'] = 3
            else:
                # 纵向保持或远离
                pass
        else:
            # segment 7 只存在一辆BV，判断motion
            if s_matrix[0][6] < -5:
                # 纵向接近
                code['motion_B'] = 3
            else:
                # 纵向保持或远离
                pass
    return [code['maneuver'], code['BV_A'], code['BV_B'], code['BV_C'], code['motion_A'], code['motion_B'],
            code['motion_C']]


def encoder_lc(mm_matrix):
    code = {
        'maneuver': 1,  # 0: LaneKeep
        'BV_A': 0,  # 0: 不存在BV, 1: 存在BV
        'BV_B': 0,  # 0: 不存在BV, 1: 存在BV
        'BV_C': 0,  # 0: 不存在BV, 1: 存在BV
        'motion_A': 0,  # LaneKeep 0: 纵向保持+横向保持, 1: 纵向接近+横向保持
        'motion_B': 0,  # 0: 纵向保持, 1: 纵向接近
        'motion_C': 0  # LaneKeep 0: 纵向保持+横向保持, 1: 纵向接近+横向保持
    }

    if mm_matrix[0][0] != -1:
        code['BV_C'] = 1
        if mm_matrix[1][2] == 'vel-keep':
            if mm_matrix[1][1] == 'lane-keep':
                code['motion_C'] = 0
            elif mm_matrix[1][1] == 'cut-out':
                code['motion_C'] = 1
            elif mm_matrix[1][1] == 'cut-in':
                code['motion_C'] = 2
        else:
            if mm_matrix[1][1] == 'lane-keep':
                code['motion_C'] = 3
            elif mm_matrix[1][1] == 'cut-out':
                code['motion_C'] = 4
            elif mm_matrix[1][1] == 'cut-in':
                code['motion_C'] = 5
    if mm_matrix[0][1] != -1:
        code['BV_A'] = 1
        if mm_matrix[1][2] == 'vel-keep':
            if mm_matrix[1][1] == 'lane-keep':
                code['motion_A'] = 0
            elif mm_matrix[1][1] == 'cut-out':
                code['motion_A'] = 1
            elif mm_matrix[1][1] == 'cut-in':
                code['motion_A'] = 2
        else:
            if mm_matrix[1][1] == 'lane-keep':
                code['motion_A'] = 3
            elif mm_matrix[1][1] == 'cut-out':
                code['motion_A'] = 4
            elif mm_matrix[1][1] == 'cut-in':
                code['motion_A'] = 5
    if mm_matrix[0][2] != -1:
        code['BV_B'] = 1
        if mm_matrix[1][2] == 'vel-keep':
            code['motion_B'] = 0
        else:
            code['motion_B'] = 3
    return [code['maneuver'], code['BV_A'], code['BV_B'], code['BV_C'], code['motion_A'], code['motion_B'],
            code['motion_C']]


source_directory = './output/pre_processed'
# output_directory = './output/code'
# header = pd.DataFrame(
#     columns=['source_file', 'track_name', 'maneuver', 'BV_A', 'BV_B', 'BV_C', 'motion_A', 'motion_B', 'motion_C', 'TTC'])
# output_file_name = output_directory + '/' + 'codes.csv'
# with open(output_file_name, 'w', newline='') as csv_file:
#     writer = csv.writer(csv_file)
#     writer.writerow(header)

# for track in os.listdir(source_directory):
    # output_file_name = output_directory + '/' + track[:-4] + '.csv'
track = 'Round_Scenario_1013.csv'
track_name = track[:-4]
source_file_path = os.path.join(source_directory, track)
df = pd.read_csv(source_file_path)

# 在“track”中找到该id的场景起始帧和结束帧
ego_track = df[df['id'] == ego_id]
ego_frame_range_actual = ego_track['frame'].unique()
ego_end_frame_index = ego_frame_range_actual[-1]
ego_init_frame_index = ego_frame_range_actual[0]
ego_init_frame = ego_track[ego_track['frame'] == ego_init_frame_index]
source_file_name = ego_init_frame['file_name']
ego_lane_change_flag = ego_track['laneId'].unique()

# 判断maneuver
if len(ego_lane_change_flag) == 1:  # 无换道
    maneuver = 'LaneKeep'
    BV_matrix, s_matrix, sequence_matrix = get_matrix(ego_track)
    code = encoder_lk(BV_matrix, s_matrix, sequence_matrix)
else:  # 发生了换道
    maneuver = 'LaneChange'
    BV_matrix, s_matrix, sequence_matrix = get_matrix(ego_track)
    mm_matrix = get_mm_matrix(df, BV_matrix, s_matrix)
    code = encoder_lc(mm_matrix)
print(BV_matrix)
print(sequence_matrix)
# 建立position， motion矩阵
#
# min_ttc = ego_track['ttc'].min()
# # 如果trackname中的第4个_后面的数字是1，则表示碰撞，TTC=0
# filename = ego_init_frame['file_name']
# parts = filename.values[0].split('_')
# if parts[4].isdigit() and int(parts[4]) == 1:
#     # 如果第4个_后面的数字是1，表示有碰撞，设定TTC为0
#     min_ttc = 0
# code.append(min_ttc)
#
# code.insert(0, track_name)
# code.insert(0, source_file_name)
# # codes.loc[len(codes)] = code

# header = ['init_frame', 'end_frame', 'ego_id', 'velocity', 'acceleration', 'ttc', 'maneuver',
#           'BV_1', 'BV_2', 'BV_3', 'BV_4', 'BV_5', 'BV_6', 'BV_7', 'BV_8',
#           'IBV_A', 's_relative_A', 'v_A', 'a_A', 'LaneChange_A',
#           'IBV_B', 's_relative_B', 'v_B', 'a_B', 'LaneChange_B',
#           'IBV_C', 's_relative_C', 'v_C', 'a_C', 'LaneChange_C',
#           'code']
# # 自车信息
# ego_init_frame = ego_track[ego_track['frame'] == ego_init_frame_index]
# velocity = math.sqrt(ego_init_frame['v_x_lane'].values[0] ** 2 + ego_init_frame['v_y_lane'].values[0] ** 2)
# acceleration = math.sqrt(ego_init_frame['acc_x_lane'].values[0] ** 2 + ego_init_frame['acc_y_lane'].values[0] ** 2)
# ttc = ego_init_frame['ttc'].values[0]
#
# result = [ego_init_frame_index, ego_end_frame_index, ego_id, velocity, acceleration, ttc, maneuver,
#           ]
#
# with open(output_file_name, 'a', newline='') as csv_file:
#     writer = csv.writer(csv_file)
#     writer.writerow(code)
