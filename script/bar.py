import matplotlib
import matplotlib.pyplot as plt
import numpy as np


# labels = ['00', '01', '02', '03']
labels = ['00', '01', '02', '03','04','05','06','07']

# plt.rcParams['font.sans-serif'] = ['SimHei']  # 中文字体设置-黑体
# plt.rcParams['axes.unicode_minus'] = False  # 解决保存图像是负号'-'显示为方块的问题

x = np.arange(len(labels))
width = 0.25
fig, ax = plt.subplots()

# color_my = ['C2','C3','C1','C6','C5','C7']
color_my = ['C1','C2','C3']


def huizhi(ax, data, x):
    bottom = 0
    for j in range(len(data)):
        height = data[j]
        ax.bar(x, height, width, bottom=bottom, color=color_my[j])
        bottom += height


data0 = [[5.91505,24.925,2.07693],
        [6.71911,34.925,2.16518] ]

data1 = [[4.89525,11.8524,2.39282],
         [5.6838,25.2616,2.15427]]

data2 = [[5.47865,22.3425,2.09964],
         [5.72069,31.6678,2.13295]]

data3 = [[7.54877,33.0234,1.76608],
         [8.62142,49.6761,2.44719]]

data4 = [[14.9622,15.2159,4.73244],
         [15.288,59.6635,4.60426]]

data5=[[14.663,18.9695,5.75863],
       [15.7146,53.6834,5.2963]]

data6 = [[16.1396,20.8607,4.25435],
         [16.4568,84.3546,5.29445]]

data7 = [[13.3536,20.8452,7.73906],
         [12.6554,45.2925,7.15426]]
para=[ -0.5, 0.5, 1.5, 2.0, 2.5, 3.0, 3.5]
# para=[-1.5, -0.5, 0.5, 1.5]


def draw_each_bar(data_num, x, labels = ['Downsample','IEKF+ICP','UpdateMap']):
    print(x)
    for i in range(len(data_num)):
        bottom = 0
        for j in range(len(data_num[i])):
            height = data_num[i][j]
            if i==1 and x == 1:
                print('运行1')
                ax.bar(x + para[i] * width, height, width, bottom=bottom, facecolor= color_my[j],alpha = 0.9,label=labels[j])
            else:
                print(x + para[i] * width)
                ax.bar(x + para[i] * width, height, width, bottom=bottom, facecolor=color_my[j], alpha=0.9)
            plt.axvline(x=x + (para[i] -0.45)* width, ls="-", c="white")  # 添加垂直直线
            bottom += height



draw_each_bar(data0, x[0])
draw_each_bar(data1, x[1])
draw_each_bar(data2, x[2])
draw_each_bar(data3, x[3])
draw_each_bar(data4, x[4])
draw_each_bar(data5, x[5])
draw_each_bar(data6, x[6])
draw_each_bar(data7, x[7])

# ax.legend(loc=2)
ax.legend(loc=2)
ax.set_ylabel('Time(ms)')
ax.set_xlabel('Sequence')
ax.set_xticks(x)
ax.set_xticklabels(labels)

#保存图片
fig.tight_layout()
# plt.savefig("测试数据.png",dpi=300)
plt.show()