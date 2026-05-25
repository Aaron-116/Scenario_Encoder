import pandas as pd


def explain_code(row):
    maneuver_map = {0: '直行', 1: '变道'}
    BV_map = {0: '不存在', 1: '存在'}
    motion_A_map = {0: '无冲突', 1: '非接近切出', 2: '非接近切入', 3: '接近不变道', 4: '接近切出', 5: '接近切入'}
    motion_B_map = {0: '无冲突', 3: '接近不变道'}
    motion_C_map = {0: '无冲突', 1: '非接近切出', 2: '非接近切入', 3: '接近不变道', 4: '接近切出', 5: '接近切入'}

    explanation = f"ADS{maneuver_map[row['maneuver']]}，BV_A{BV_map[row['BV_A']]}，BV_B{BV_map[row['BV_B']]}，BV_C{BV_map[row['BV_C']]}，BV_A{motion_A_map[row['motion_A']]}，BV_B{motion_B_map[row['motion_B']]}，BV_C{motion_C_map[row['motion_C']]}"
    return explanation


# 读取CSV文件
df = pd.read_csv('../output/code_260525/codes.csv')
# 过滤掉 BV_A、BV_B、BV_C 都为0的行
df = df[(df['BV_A'] != 0) | (df['BV_B'] != 0) | (df['BV_C'] != 0)]
# 提取前 5000 行
df_top_5000 = df.iloc[:5000]
# 筛选出需要删除的行
rows_to_drop = df_top_5000[(df_top_5000['maneuver'] == 0) & (df_top_5000['BV_A'] == 0) & (df_top_5000['BV_B'] == 1) & (
            df_top_5000['BV_C'] == 0)]
# 获取需要删除的行索引
rows_to_drop_indices = rows_to_drop.index
# 从原始 DataFrame 中删除这些行
df = df.drop(rows_to_drop_indices)

# 统计机动类型的场景个数
maneuver_stats = df['maneuver'].value_counts()
print("机动类型统计:")
print(maneuver_stats)

# 统计背景车数量
df['bgv_count'] = df[['BV_A', 'BV_B', 'BV_C']].sum(axis=1)
bgv_count_stats = df['bgv_count'].value_counts()
print("背景车数量统计:")
print(bgv_count_stats)

# 将A-G列的编码组合成一个字符串，作为编码案例的唯一标识
df['code'] = df[['maneuver', 'BV_A', 'BV_B', 'BV_C', 'motion_A', 'motion_B', 'motion_C']].apply(
    lambda row: ''.join(row.values.astype(str)), axis=1)

# 统计不同编码案例的种类
code_counts = df['code'].value_counts()
# 计算每一类编码案例占总数的比例
code_percentages = code_counts / code_counts.sum()
row_count = len(df)
row_count_df = pd.DataFrame({'总数': [row_count]})

# 将统计信息保存到CSV文件
stats_df = pd.DataFrame({
    'ADS机动类型': maneuver_stats.index,
    '场景计数': maneuver_stats.values
})

bgv_stats_df = pd.DataFrame({
    '场景中IBV数量': bgv_count_stats.index,
    '场景计数': bgv_count_stats.values
})

df['code_explanation'] = df.apply(explain_code, axis=1)

# 统计不同编码案例的种类
code_counts = df['code'].value_counts()

# 对 code_counts 进行降序排序
code_counts_sorted = code_counts.sort_values(ascending=False)

# 创建 code_stats_df
code_stats_df = pd.DataFrame({
    '编码案例': code_counts_sorted.index,
    '编码案例数量': code_counts_sorted.values,
    '编码案例比例(%)': code_percentages.values
})

# 添加编码案例解释
code_explanation_stats = df.groupby('code')['code_explanation'].first().reset_index()
code_explanation_dict = dict(zip(code_explanation_stats['code'], code_explanation_stats['code_explanation']))
code_stats_df['编码案例解释'] = code_stats_df['编码案例'].map(code_explanation_dict)

df['ttc_less_than_1'] = (df['TTC'] >= 0) & (df['TTC'] < 1)
ttc_stats = df.groupby('code')['ttc_less_than_1'].agg(['sum', 'size'])
ttc_stats['percentage'] = (ttc_stats['sum'] / ttc_stats['size'])
code_stats_df = code_stats_df.merge(ttc_stats, left_on='编码案例', right_index=True, how='left')
code_stats_df.rename(columns={'sum': 'TTC小于1的数量', 'size': '总场景数', 'percentage': 'TTC小于1的比例(%)'},
                     inplace=True)

# 合并所有统计数据
result_df = pd.concat([row_count_df, stats_df, bgv_stats_df, code_stats_df], axis=1)

# 将结果保存到CSV文件
result_df.to_csv('../output/code/stats_output.csv', index=False)
