import matplotlib.pyplot as plt

# 数据
labels = ['1 IBV', '2 IBVs', '3 IBVs']
sizes1 = [72.9, 25.5, 1.6]  # 第一个饼图的数据
sizes2 = [1588, 2004, 808]  # 第二个饼图的数据
colors = [(142/255, 182/255, 156/255), (237/255, 221/255, 195/255), (238/255, 191/255, 109/255)]  # 各部分的颜色

# 创建图形和子图
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 3))

# 绘制第一个饼图
wedges1,  autotexts1 = ax1.pie(
    sizes1,
    colors=colors,
    # autopct='%1.1f%%',  # 显示百分比，保留一位小数
    startangle=140,  # 起始角度
    # textprops={'color': 'black', 'fontsize': 12},  # 设置文字颜色和大小
    wedgeprops=dict(edgecolor='white', linewidth=0.5)
)

# 绘制第二个饼图
wedges2,  autotexts2 = ax2.pie(
    sizes2,
    colors=colors,
    # autopct='%1.1f%%',  # 显示百分比，保留一位小数
    startangle=140,  # 起始角度
    # textprops={'color': 'black', 'fontsize': 12}  # 设置文字颜色和大小
    wedgeprops=dict(edgecolor='white', linewidth=0.5)
)

# 调整子图间距
plt.subplots_adjust(wspace=0)  # 水平间距为 0.3

# 添加图例
fig.legend(
    wedges1,  # 使用 wedges1 作为图例的来源
    labels,  # 使用 labels 作为图例的标签
    loc="center right",  # 图例位置
    # bbox_to_anchor=(0, 0),  # 调整图例的位置
    fontsize=12  # 设置图例字体大小
)

# 展示图形
# plt.tight_layout()
plt.savefig('./png/背景车数量统计.png', dpi=300)
plt.show()