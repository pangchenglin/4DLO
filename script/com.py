import numpy as np
import matplotlib.pyplot as plt
import pandas as pd #使用该数据包导入数据表
import random
import matplotlib as mpl


steam_dicp = [118.86,  95.6565, 118.505,131.27,238.876,223.184,243.712,185.237]
Dvoxelmap = [43.556,33.5326,42.6627,61.9876,86.1917,79.8522,113.561,67.1237]
ours = [27.7465,24.2551, 26.3421,40.32,32.711,32.6667,35.556,28.8381]

A = [[28, 3, 5, 6, 3, 7, 2, 4, 9, 95],
     [58, 6, 13, 13, 5, 12, 17, 11, 10, 102],
     [60, 22, 21, 41, 11, 5, 1, 2, 3, 4]]

x_labels = ["00","01","02","03","04","05","06","07"]
# for item in range(0, 100, 10):
#    x = item + 10
#    if x == 10:
#        x_labels.append("{}~{}".format(0, 10))
#    else:
#        x_labels.append("{}".format(x))

x = np.arange(8)
# 生成多柱图
plt.bar(x + 0.00, ours, color='orange', width=0.3, label="OURS")
plt.bar(x + 0.30, Dvoxelmap, color='royalblue', width=0.3, label="DVoxelMap")
plt.bar(x + 0.60, steam_dicp, color='brown', width=0.3, label="STEAM-DICP")
# 图片名称
# plt.title('多柱图')
# 横坐标绑定
plt.xlabel("Sequences",fontsize=10)
plt.ylabel("Time(ms)",fontsize=10)
plt.xticks(x + 0.30, x_labels)
# 生成图片
plt.legend(loc="best")
# plt.savefig("多柱图.png", dpi=700, fontproperties=prop)
plt.show()