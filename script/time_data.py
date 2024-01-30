import csv
import matplotlib.pyplot as plt
path1 = "/home/pclin/WorkSpace/S-RTS_aeva/src/CT-RTS/output/time/time_our07.csv"
path2 = "/home/pclin/WorkSpace/S-RTS_aeva/src/CT-RTS/output/time/time_voxel07.csv"

data1 = []
num1 = []
data2 = []
num2 = []

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
    # fig, ax = plt.subplots(figsize=(6, 6), layout='constrained')
    plt.subplot(2, 1, 1)
    plt.ylabel("Time(ms)")

    plt.plot(num1, data1, linewidth=0.3)
    plt.subplot(2, 1, 2)
    plt.plot(num2, data2, linewidth=0.3)
    plt.xlabel("Frame Number")
    plt.ylabel("Time(ms)")
    plt.show()
    # ax.plot(num1,data1,label='OURS')
    # ax.plot(num2,data2,label='DVoxelMap')
    # # print(data1)    
    # # plt.plot(num1,data1)
    # plt.xlabel("Frame Number")
    # plt.ylabel("Time(ms)")
    # ax.legend(loc="lower left");  # 显示图例，对应ax.plot()中的lable属性, loc指定图例显示在图的哪个位置
    # fig.show()