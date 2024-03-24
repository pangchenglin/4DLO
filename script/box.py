
import matplotlib.pyplot as plt
import numpy as np
import csv


path1 = "/home/pclin/CT-RTS/output/time/time_our07.csv"
path2 = "/home/pclin/CT-RTS/output/time/time_voxel07.csv"

# 生成示例数据
def readpath(path):
    num = []
    data = []
    with open(path,'r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            num.append(int(row[0]))
            data.append(float(row[2]))
    return num,data
 
 
if __name__ == "__main__":
    num1,data1 = readpath(path1)
    num2,data2 = readpath(path2)
# 绘制箱体图
    labels = ['OURS','DVoxelMap']
    fig, ax = plt.subplots()
    ax.boxplot([data1,data2],sym=' ')
 
# 设置标题和轴标签
    ax.set_title('Boxplot Example')
    ax.set_xlabel('dataset')
    ax.set_ylabel('time')
    ax.set_xticks([1,2],['OUR','DVoxelMap'])
    plt.show()