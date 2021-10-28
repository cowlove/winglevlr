#!/usr/bin/python3

# about 2 seconds for 3200 second data file 

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from time import sleep

# global map of column names vs column number and axes number 
colnames = [
   "time", "hdgGPS", "hdgGDL90", "hdgVTG", "hdgRMC", "galt", "ax", "ay", "az", "gx", "gy", "gz", 
   "mx", "my", "mz", "dtk", "g5track", "palt", "gspeed", "g5pitch", "g5roll", "g5hdg", "g5ias", 
   "g5tas", "g5palt", "g5slip", "ubloxhdg", "ubloxha", "ubloxalt", "ubloxgs", "lat", "lon",
   "servo0", "servo1", "flag", "desroll", "roll", "maghdg", "xte", "pitch", "dalt", "dpitch", 
   "stickX", "stickY", 
]

ax2cols = ["hdgGPS", "hdgGDL90", "hdgVTG", "hdgRMC", "g5hdg", "ubloxhdg", "lat", "lon", "servo0", 
    "servo1"]

# add specified columns from the data file to global ax1, ax2 exes 
def addfile(fname, cols, diffArray = []): 
    global ax1, ax2
    dcn = [0,] # column numbers in the data file 
    for x in map(lambda i: colnames.index(i), cols):
        dcn += [x, ]

    array  = np.loadtxt(fname, usecols=dcn)
    if len(diffArray) == len(array[:,1]):
        array[:,1] = [(a - b) for a, b in zip(array[:,1], diffArray)]
    for i in range(1, len(dcn)):
        try: 
            ax2cols.index(cols[i - 1])
            ax2.plot(array[:, 0], array[:, i], label=cols[i - 1])
        except:
            ax1.plot(array[:, 0], array[:, i], label=cols[i - 1])

def setDiff(fname, col): 
    return np.loadtxt(fname, usecols=[colnames.index(col),])

# add specified columns from the data file to global x1 axis in an X vs Y scatter
def addfileXY(fname, cols): 
    global ax1, ax2
    dcn = [] 
    for x in map(lambda i: colnames.index(i), cols):
        dcn += [x, ]

    array  = np.loadtxt(fname, usecols=dcn)
    ax1.plot(array[:, 0], array[:, 1], label=cols[0] + " vs " + cols[1])


mpl.rcParams["lines.linewidth"] = .5
fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

plt.xlim([2000, 2500])

#addfile("./logs/AHRSD044.plog", ["pitch", "g5pitch"]) 
diffArray = setDiff("./logs/AHRSD044.plog", "g5pitch")
addfile("./logs/AHRSD044.plog", ["pitch"], diffArray) 

#addfileXY("./logs/AHRSD044.plog", ["lat", "lon"]) # -map
#addfileXY("./logs/AHRSD044.plog", ["mx", "my"])  # -magplot

for a in [ax1, ax2]:
    a.margins(0,0)
    a.legend()
ax2._get_lines.prop_cycler = ax1._get_lines.prop_cycler
ax1.grid(b=True, axis="both", alpha=.3) 
plt.get_current_fig_manager().full_screen_toggle()
plt.subplots_adjust(left=0.03, right=0.97, top=0.97, bottom=0.03)
plt.show(block=True)




