#
# filename: test_sensor.py
#
# description: This file takes the test data for the sensor and runs it through a test. 
#
# Written by: Marek Newton
#

import pandas as pd
import numpy as np

import matplotlib.pyplot as plt

# load csv data into a dataframe
voltage_dataframe = pd.read_csv('voltage_data.csv')
current_dataframe = pd.read_csv('current_data.csv')

# convert the data frames to numpy arrays
voltage_data = voltage_dataframe.to_numpy()
current_data = current_dataframe.to_numpy()

# find the best fit line for the voltage data
voltage_fit = np.polyfit(voltage_data[:,0], voltage_data[:,1], 1)
voltage_fit_fn = np.poly1d(voltage_fit)

# find the best fit line for the current data
current_fit = np.polyfit(current_data[:,0], current_data[:,1], 1)
current_fit_fn = np.poly1d(current_fit)

def percent_error(actual, expected):
    return abs(actual - expected) / expected

# calculate the linearity error for the voltage data based on the best fit line
voltage_linearity_error = 0
for i in range(len(voltage_data)):
    voltage_linearity_error += percent_error(voltage_data[i,1], voltage_fit_fn(voltage_data[i,0]))

print(voltage_linearity_error)
voltage_linearity_error /= len(voltage_data)

# calculate the linearity error for the current data based on the best fit line
current_linearity_error = 0
for i in range(len(current_data)):
    current_linearity_error += percent_error(current_data[i,1], current_fit_fn(current_data[i,0]))

current_linearity_error /= len(current_data)

print('Voltage Linearity Percent Error: ' + str(voltage_linearity_error*100))
print('Current Linearity Percent Error: ' + str(current_linearity_error*100))

plt.figure()
# plot the voltage data
plt.scatter(voltage_data[:,0], voltage_data[:,1])
# plot the best fit line
plt.plot(voltage_data[:,0], voltage_fit_fn(voltage_data[:,0]), 'r')
# add m and b values to the plot
plt.text(0.5, 0.5, 'm = ' + str(voltage_fit[0]) + '\nb = ' + str(voltage_fit[1]))
plt.xlabel('Input Voltage (V)')
plt.ylabel('Output Voltage (V)')
plt.title('Voltage Data')
# save the plot in a plots folder
plt.savefig('plots/voltage_transfer_function.png')

plt.figure()
# plot the current data
plt.scatter(current_data[:,0], current_data[:,1])
# plot the best fit line
plt.plot(current_data[:,0], current_fit_fn(current_data[:,0]), 'r')
# add m and b values to the plot
plt.text(0.5, 0.5, 'm = ' + str(current_fit[0]) + '\nb = ' + str(current_fit[1]))
plt.xlabel('Input Current (A')
plt.ylabel('Output Voltage (V)')
plt.title('Current Data')
plt.savefig('plots/current_transfer_function.png')
