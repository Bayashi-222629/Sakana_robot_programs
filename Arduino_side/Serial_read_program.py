from itertools import count
import serial
import time
import pandas as pd
import numpy as np
import csv

output = [[]]
ser.close()
ser = serial.Serial('COM5', 9600, timeout=0.1) 
print("start")

for i in range(500):

    data = ser.readline().decode('utf8') #utf8でデータを読み込む
    data_array= data.split(",")          #「,」で区切る。
    print(data_array)
    output.append(data_array)    #data_array配列に上から書き込んでいく
    time.sleep(0.01)

ser.close()

output = pd.DataFrame(output)          #DataFrame形式に変換する
output.to_csv("data_FF.csv")           #“”内の名前でCSVファイルを作る
print(output)
print("end")
