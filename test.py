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