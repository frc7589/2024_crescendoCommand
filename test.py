"""
import sys
import time
from networktables import NetworkTables
import logging
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv

filePath = r'C:\frc\2024_crescendoCommand\points.csv'
points = []

# 初始化數據
x_data = []
y_data = []

def quadratic(x, a, b, c):
    return a * x**2 + b * x + c

def update(new_x, new_y):
    print(new_x, new_y)
    x_data.append(new_x)
    y_data.append(new_y)

    x_data_np = np.array(x_data)
    y_data_np = np.array(y_data)

    # 確保有足夠的數據點
    if len(x_data_np) < 3:
        return

    # 擬合二次曲線
    params, _ = curve_fit(quadratic, x_data_np, y_data_np)
    print(params)
    
    sd.putNumber("a", params[0])
    sd.putNumber("b", params[1])
    sd.putNumber("c", params[2])

ip = "10.75.89.2"

NetworkTables.initialize(server=ip)

sd = NetworkTables.getTable("SmartDashboard")

with open(filePath, 'r') as f:
    csvReader = csv.reader(f)
    rows = list(csvReader)

    for point in rows:
        #print("add")
        update(point[0], point[1])
        points.append(point)

    #print(x_data)
    #print(y_data)

with open(filePath, 'a+') as f:\

    while True:
        point = sd.getNumberArray("dataPoint", [x_data[0], y_data[0]])
        if point not in points:
            print("add")
            print(f.write(f"{point[0]},{point[1]}\r\n"))
            update(point[0], point[1])
            points.append(point)

        time.sleep(1)
"""
import sys
import time
from networktables import NetworkTables
import logging
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv

filePath = r'C:\frc\2024_crescendoCommand\points.csv'
points = []

# 初始化數據
x_data = []
y_data = []

# 四次多項式回歸函數
def quartic(x, a, b, c, d, e):
    return a * x**4 + b * x**3 + c * x**2 + d * x + e

def update(new_x, new_y):
    print(new_x, new_y)
    x_data.append(new_x)
    y_data.append(new_y)

    x_data_np = np.array(x_data)
    y_data_np = np.array(y_data)

    # 確保有足夠的數據點
    if len(x_data_np) < 5:
        return

    # 擬合四次多項式
    params, _ = curve_fit(quartic, x_data_np, y_data_np)
    print(params)
    
    sd.putNumber("a", params[0])
    sd.putNumber("b", params[1])
    sd.putNumber("c", params[2])
    sd.putNumber("d", params[3])
    sd.putNumber("e", params[4])

ip = "10.75.89.2"

NetworkTables.initialize(server=ip)

sd = NetworkTables.getTable("SmartDashboard")

with open(filePath, 'r') as f:
    csvReader = csv.reader(f)
    rows = list(csvReader)

    for point in rows:
        # 確保數據是浮點數
        x = float(point[0])
        y = float(point[1])
        update(x, y)
        points.append(point)

    # 如果需要，可以在这里绘制数据和回归曲线
    # x_data_np = np.array(x_data)
    # y_data_np = np.array(y_data)
    # params, _ = curve_fit(quartic, x_data_np, y_data_np)
    # plt.plot(x_data_np, y_data_np, 'bo', label='Data points')
    # plt.plot(x_data_np, quartic(x_data_np, *params), 'r-', label='Fitted curve')
    # plt.legend()
    # plt.show()

with open(filePath, 'a+') as f:
    while True:
        point = sd.getNumberArray("dataPoint", [x_data[0], y_data[0]])
        if point not in points:
            print("add")
            f.write(f"{point[0]},{point[1]}\r\n")
            update(point[0], point[1])
            points.append(point)

        time.sleep(1)
