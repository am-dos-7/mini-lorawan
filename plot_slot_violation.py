#! /bin/python3 

# Date: 24/08/2023
# @author: Amavi DOSSA
# Description: will read an input file containing slot-Aloha over LoRaWAN
# experiment stats and plot the evolution of slot violation in time. 


from typing import final
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Transmission parameters (time)
T_TX = 306    # ms
T_RX = 91     # ms
T_B1 = 180     # ms
T_B2 = 180     # ms


# file_name = "slot_violation_first_test.txt"
file_name = "slot_violation_new.txt"
data = pd.read_csv(file_name)
print("len: ", len(data), sep='')
print(data.head)

time = data['time']
pos = data['pos']


x = np.zeros(len(time))     # Time translation (from 0 to 'time[len(time)]-time[0]')
upper_bound = np.zeros(len(time))
lower_bound = np.zeros(len(time))
ideal_arrival = np.zeros(len(time))

for i in range(len(time)):
    x[i] = time[i] - time[0]
    upper_bound[i] = T_TX + T_B1 + T_B2
    lower_bound[i] = T_TX
    ideal_arrival[i] = T_TX + T_B1

# x = x*1000

pos_arr = pos.to_numpy()    # convert pos DataFrame to numpy array for the plot function

plt.plot(x, pos_arr, x, upper_bound, x, lower_bound, x, ideal_arrival)
plt.xlabel("Time (s)", )
plt.ylabel("Packet arrival (ms)")
plt.legend(['Node sync', 'Upper bound', 'Lower bound', 'Ideal arrival'])
plt.show()
