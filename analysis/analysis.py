import pandas as pd

# 读取CSV文件
df = pd.read_csv('../output/code/codes.csv')

# 统计机动类型的场景个数
maneuver_stats = df['maneuver'].value_counts()
print("机动类型统计:")
print(maneuver_stats)

# 计算每行中背景车的存在数量
df['bgv_count'] = df[['BV_A', 'BV_B', 'BV_C']].sum(axis=1)
# 统计包含0、1、2、3辆背景车的场景个数
bgv_count_stats = df['bgv_count'].value_counts()
print("背景车数量统计:")
print(bgv_count_stats)



# 筛选出maneuver=0的场景
df_maneuver_0 = df[df['maneuver'] == 0]
# 进一步筛选出只存在BV_A的场景
s1 = df_maneuver_0[(df_maneuver_0['BV_A'] == 1) &
                          (df_maneuver_0['BV_B'] == 0) &
                          (df_maneuver_0['BV_C'] == 0)]
s1_count = s1.shape[0]
print("直行，前方1辆BV，筛选结果:")
print(s1_count)



# 进一步筛选出存在BV_C的场景
df_result = df_maneuver_0[(df_maneuver_0['BV_A'] == 1) &
                          (df_maneuver_0['BV_B'] == 1) &
                          (df_maneuver_0['BV_C'] == 1)]
# 显示结果
print("直行，3辆IBV，筛选结果:")
print(df_result)



# 筛选出maneuver=1的场景
df_maneuver_1 = df[df['maneuver'] == 1]
# 进一步筛选出存在BV_C的场景
df_result1 = df_maneuver_1[(df_maneuver_1['BV_A'] == 1) &
                          (df_maneuver_1['BV_B'] == 1) &
                          (df_maneuver_1['BV_C'] == 1)]
# 显示结果
print("变道，3辆IBV，筛选结果:")
print(df_result1)