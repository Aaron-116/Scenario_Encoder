import os
import pandas as pd

source = 'D:\Group\Github\Scenario_Encoder\dataset\AD4CHE_Data_V1.0'
code = pd.DataFrame(columns=['code'])
for file in os.listdir(source):
    code_path = os.path.join(source, file, 'scenario_code.csv')
    # 读取scenario_code.csv文件，并将'code'添加到code列表中
    df = pd.read_csv(code_path)
    code = pd.concat([code, df[['code']]], ignore_index=True)
# 将code列表转换为DataFrame，并保存为csv文件
code.to_csv('./code.csv', index=False)

# 读取CSV文件
df = pd.read_csv('./code.csv')

# 提取第二位数字
df['second_digit'] = df['code'].str[1]

# 统计第二位数字为1和2的数量
count_1 = df[df['second_digit'] == '1'].shape[0]
count_2 = df[df['second_digit'] == '2'].shape[0]

# 计算比例
total = df.shape[0]
ratio_1 = count_1 / total
ratio_2 = count_2 / total

print(f"第二位数字为1的比例: {ratio_1:.2%}")
print(f"第二位数字为2的比例: {ratio_2:.2%}")