from tkinter import Tk
from tkinter.filedialog import askopenfilename
import matplotlib.pyplot as plt
import csv

plot_list = ['servo0','servo1','servo2','servo3','servo4','servo5'];

# frequency = input("At what input rate would you like the final output (hz): ")

Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
filename = askopenfilename() # show an "Open" dialog box and return the path to the selected file

data = {}
time = {}

with open(filename) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=",")
    line_count = 0;
    for row in csv_reader:
        if line_count != 0:
            if row[1] not in list(data.keys()):
                time[row[1]] = [float(row[2])]
                data[row[1]] = [float(row[3])]
            else:
                time[row[1]].append(float(row[2]))
                data[row[1]].append(float(row[3]))
        line_count += 1;
    print(f'Processed {line_count-1} lines.')

minimum_time = -1;
for key in list(time.keys()):
    for t in time[key]:
        if minimum_time < 0:
            minimum_time = t;
        elif minimum_time > t:
            minimum_time = t;
for key in list(time.keys()):
    time[key] = [t-minimum_time for t in time[key]]



plt.figure()
x = 1
y = len(plot_list)
i = 1;
for type in list(data.keys()):
    if type in plot_list:
        ax = plt.subplot(y,x,i)
        plt.plot(time[type],data[type])
        ax.set_ylabel(type)
        i += 1
ax.set_xlabel("t - time (s)")
plt.show()
