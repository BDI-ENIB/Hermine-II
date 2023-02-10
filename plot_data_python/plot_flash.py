# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import numpy as np
import matplotlib.pyplot as plt

file=open("input.txt","r")

lines = file.readlines()
count=0
data1 = np.array(np.zeros(len(lines)))
data2 = np.array(np.zeros(len(lines)))
data3 = np.array(np.zeros(len(lines)))
data4 = np.array(np.zeros(len(lines)))
data5 = np.array(np.zeros(len(lines)))
data6 = np.array(np.zeros(len(lines)))
data7 = np.array(np.zeros(len(lines)))
data8 = np.array(np.zeros(len(lines)))
data9 = np.array(np.zeros(len(lines)))
#xpoint = np.array(np.zeros(len(text)))

for line in lines:
    data1[count] = line.split(",")[1]
    data2[count] = line.split(",")[2]
    data3[count] = line.split(",")[3]
    data4[count] = line.split(",")[4]
    data5[count] = line.split(",")[4]
    data6[count] = line.split(",")[6]
    data7[count] = line.split(",")[7]
    data8[count] = line.split(",")[8]
    data9[count] = line.split(",")[9]
    count+=1



plt.plot(data1)
plt.plot(data2)
plt.plot(data3)
plt.show()

plt.plot(data4)
plt.plot(data5)
plt.plot(data6)
plt.show()


plt.plot(data8)
plt.show()

plt.plot(data9)
plt.show()