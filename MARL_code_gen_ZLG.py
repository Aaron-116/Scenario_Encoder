import math

import numpy as np
import pandas as pd
import csv
import os


def get_matrix(ego_track):
    # 定义8个segment的BV_id和s_relative, 分别存储在BV_matrix和s_matrix中, 每个segment最多记录2辆BV
    BV_matrix = -1 * np.ones((2, 9))
    s_matrix = np.nan * np.empty((2, 9))
    # 记录BV的出现顺序
    sequence_matrix = np.zeros((4, 9))
    # 遍历所有segment
    for i in range(0, 10):
        # 获取当前segment中所有BV的id
        if i == 9:
            data = ego_track['BV_0']
        else:
            data = ego_track[f'BV_{i}']

        # 判断出现顺序
        sequence_matrix[0][i - 1] = data[0]
        k = 1
        for j in range(1, len(data)):
            if data[j] != data[j - 1]:
                sequence_matrix[k][i - 1] = data[j]
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
            BV_matrix[0][i - 1] = data_unique[0]
        # 如果有两辆BV, 则将其id存储在BV_matrix中，先出现的会先记录
        elif len(data_unique) == 2:
            BV_matrix[0][i - 1] = data_unique[0]
            BV_matrix[1][i - 1] = data_unique[1]
        # 如果有三辆以上BV, 则跳过当前segment
        else:
            continue

        # 获取当前segment中所有BV的相对位置
        if len(data_unique) == 1:
            data1 = ego_track[f's{i}'].dropna()
            # 获取最小值的索引
            min_index = data1.idxmin()
            start_index = data1.index[0]
            # 获取最小值与初始值的差
            s_relative = data1[min_index] - data1[start_index]
            s_matrix[0][i - 1] = s_relative
        elif len(data_unique) == 2:
            BV_s2 = pd.DataFrame(columns=['BV_id', 's_relative'])
            BV_s3 = pd.DataFrame(columns=['BV_id', 's_relative'])
            for BV in ego_track[f'BV_{i}']:
                if BV == data_unique[0]:
                    data2 = ego_track[ego_track[f'BV_{i}'] == BV][f's{i}'].dropna()
                    BV_s2.loc[len(BV_s2)] = [BV, data2]
                elif BV == data_unique[1]:
                    data3 = ego_track[ego_track[f'BV_{i}'] == BV][f's{i}'].dropna()
                    BV_s3.loc[len(BV_s3)] = [BV, data3]
            min_index2 = BV_s2['s_relative'].idxmin()
            start_index2 = BV_s2['s_relative'].index[0]
            s_relative2 = BV_s2['s_relative'][min_index2] - BV_s2['s_relative'][start_index2]
            min_index3 = BV_s3['s_relative'].idxmin()
            start_index3 = BV_s3['s_relative'].index[0]
            s_relative3 = BV_s3['s_relative'][min_index3] - BV_s3['s_relative'][start_index3]
            s_matrix[0][i - 1] = s_relative2
            s_matrix[1][i - 1] = s_relative3
    return BV_matrix, s_matrix, sequence_matrix


def encoder(maneuver, BV_matrix, s_matrix, sequence_matrix):
    code = {
        'maneuver': 0,  # 0: LaneKeep, 1: LaneChange
        'BV_A': 0,      # 0: 不存在BV, 1: 存在BV
        'BV_B': 0,      # 0: 不存在BV, 1: 存在BV
        'BV_C': 0,      # 0: 不存在BV, 1: 存在BV
        'motion_A': 0,  # 0: 纵向保持, 1: 纵向接近
        'motion_B': 0,  # 0: 纵向保持, 1: 纵向接近
        'motion_C': 0   # 0: 纵向保持, 1: 纵向接近
    }
    if maneuver == 'LaneKeep':
        code['maneuver'] = 0
        if BV_matrix[0][1] != -1:
            # segment 2 存在BV
            # 获取segment 2 中BV的出现顺序
            sequence_2 = sequence_matrix[:, 1]
            sequence_2 = sequence_2[sequence_2 != 0]
            if BV_matrix[1][1] != -1:
                # segment 2 出现第二辆BV
                code['BV_C'] = 1
            else:
                # segment 2 只存在一辆BV，判断motion
                if len(sequence_2) == 1:
                    # BV一直存在,判断运动
                    code['BV_A'] = 1
                    if s_matrix[0][1] < -5:
                        # 纵向接近
                        code['motion_A'] = 1
                    else:
                        # 纵向保持或远离
                        pass
                elif len(sequence_2) == 2:
                    # BV在某个时间点出现
                    if sequence_2[0] == -1:
                        # 切入
                        code['BV_A'] = 0
                        code['BV_C'] = 1
                        # TODO: 判断BV的运动

        if BV_matrix[0][6] != -1:
            # segment 7 存在BV
            # 获取segment 7 中BV的出现顺序
            code['BV_B'] = 1
            if BV_matrix[1][6] != -1:
                # segment 7 出现第二辆BV
                if s_matrix[1][6] < -5:
                    # 纵向接近
                    code['motion_B'] = 1
                else:
                    # 纵向保持或远离
                    pass
            else:
                # segment 7 只存在一辆BV，判断motion
                if s_matrix[0][6] < -5:
                    # 纵向接近
                    code['motion_B'] = 1
                else:
                    # 纵向保持或远离
                    pass


        if BV_matrix[0][6] is not None:
            code[2] = 1
    elif maneuver == 'LaneChange':
        # TODO: 实现LaneChange分类
        pass


    return [code['maneuver'], code['BV_A'], code['BV_B'], code['BV_C'], code['motion_A'], code['motion_B'], code['motion_C']]


def detect_IBVs(flag, df):
    # 检测初始帧的IBVs
    IBVs = pd.DataFrame(columns=['IBV', 's_relative', 'v', 'a', 'LaneChange'])
    if flag == 'LaneKeep':
        # 确定 segment 7
        IBV_C = df[df['BV_7'] is not None]['id'].unique()

        return None
    elif flag == 'LaneChange':
        return None


source_directory = './output/pre_processed'
output_directory = './output/code'
codes = pd.DataFrame(columns=['maneuver', 'BV_A', 'BV_B', 'BV_C', 'motion_A', 'motion_B', 'motion_C'])

for track in os.listdir(source_directory):
    output_file_name = output_directory + '/' + track[:-4] + '.csv'
    source_file_path = os.path.join(source_directory, track)
    df = pd.read_csv(source_file_path)

    # 在“track”中找到该id的场景起始帧和结束帧
    ego_track = df[df['id'] == 15]
    ego_frame_range_actual = ego_track['frame'].unique()
    ego_end_frame_index = ego_frame_range_actual[-1]
    ego_init_frame_index = ego_frame_range_actual[0]
    ego_lane_change_flag = ego_track['laneId'].unique()

    # 判断maneuver
    if len(ego_lane_change_flag) == 1:  # 无换道
        maneuver = 'LaneKeep'
        # 建立position， motion矩阵
        BV_matrix, s_matrix, sequence_matrix = get_matrix(ego_track)
        code = encoder(maneuver, BV_matrix, s_matrix, sequence_matrix)

    else:  # 发生了换道
        maneuver = 'LaneChange'
        continue

    codes.loc[len(codes)] = code

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
    # result = [ego_init_frame_index, ego_end_frame_index, 15, velocity, acceleration, ttc, maneuver,
    #           ]
    #
    # with open(output_file_name, 'w', newline='') as csv_file:
    #     writer = csv.writer(csv_file)
    #     writer.writerow(header)
    #
    # with open(output_file_name, 'a', newline='') as csv_file:
    #     writer = csv.writer(csv_file)
    #     writer.writerow(result)
