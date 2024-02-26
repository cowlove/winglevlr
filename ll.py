#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from time import sleep
from sys import argv
import re
import os.path
import os

# Python replacement of loglook.sh
# ./ll.py 044 roll -git abcdef roll 
# -replay 044 or -replay logs/AHRSD044.DAT   re-run simulation
# -reuse 044 or -replay logs/AHRSD044.DAT    use originally logged data
# -git abcdef  Use git version abcdef for following data 
# -args "args"  Use winglevlr_ubuntu args for following data
# -diff g5hdg   Use data column as difference for next subsequent plotspec 
#            ./ll.py 044 -diff roll -git abcdef roll    Plots diff between roll in two git versios
# map         Aritificial plotspec for plotting lat vs long
# magplot     Artificial plotspec for plotting mx vs my
#
# ./ll.py 044 roll -diff maghdg -git 479eecc7b4 maghdg  
# ./ll.py -replay 044 

# global map of column names vs column number and axes number 
colnames = [
   "time", "hdgGPS", "hdgGDL90", "hdgVTG", "hdgRMC", "galt", "ax", "ay", "az", "gx", "gy", "gz", 
   "mx", "my", "mz", "dtk", "g5track", "palt", "gspeed", "g5pitch", "g5roll", "g5hdg", "g5ias", 
   "g5tas", "g5palt", "g5slip", "ubloxhdg", "ubloxha", "ubloxalt", "ubloxgs", "lat", "lon",
   "servo0", "servo1", "flag", "desroll", "roll", "maghdg", "xte", "pitch", "dalt", "dpitch", 
   "stickX", "stickY", 
]

# columns to place in the second Y axis
ax2cols = ["hdgGPS", "hdgGDL90", "hdgVTG", "hdgRMC", "g5hdg", "ubloxhdg", "lat", "lon", "servo0", 
    "servo1", "maghdg", "stickX", "stickY"]

# columns to place in the second Y axis
negcols = ["roll"]

# add specified columns from the data file to global ax1, ax2 exes 
def addfile(fname, cols, diffArray = []): 
    global ax1, ax2
    dcn = [0,] # column numbers in the data file 
    for x in map(lambda i: colnames.index(i), cols):
        dcn += [x, ]

    print(dcn)
    print(fname)
    array  = np.loadtxt(fname, usecols=dcn)
    if len(diffArray) == len(array[:,1]):
        array[:,1] = [(a - b) for a, b in zip(array[:,1], diffArray)]
    for i in range(1, len(dcn)):
        l=cols[i - 1]
        if len(diffArray) and i == 1:
            l += " (DIFFERENCE)"
        try: 
            ax2cols.index(cols[i - 1])
            ax2.plot(array[:, 0], array[:, i], label=l)
        except:
            ax1.plot(array[:, 0], array[:, i], label=l)



def evalSphere(c, array):
    dists = []
    for l in array:
        d = np.sqrt((c[0] - l[0]) * (c[0] - l[0]) + 
            (c[1] - l[1]) * (c[1] - l[1]) +
            (c[2] - l[2]) * (c[2] - l[2]))
        dists += [d, ]
    r = np.std(dists)
    return r

# add specified columns from the data file to global x1 axis in an X vs Y scatter
def magCalibrate(fname): 
    dcn = [] 
    ofile = makeFile(fname, 0, "", "")
    for x in map(lambda i: colnames.index(i), ["mx", "my", "mz"]):
        dcn += [x, ]
    array  = np.loadtxt(ofile, usecols=dcn)
    #center = [np.average(array.T[0]),np.average(array.T[1]),25]
    center = [26,37,15]
#[[ 3.56402466 26.17604815 37.20987549 25.        ]

    span = 5
    step = 1
    results = []
    for x in range(-span, span, step):
        for y in range(-span, span, step):
            print(x, y)
            for z in range(-span, span, step):
                c = [center[0] + x, center[1] + y, center[2] + z]
                r = evalSphere(c, array)
                line = [r, c[0], c[1], c[2]]
                #print (line)
                if len(results) == 0:
                    results = line
                else:
                    results = np.vstack([results, line])
    print (results[results[:,0].argsort()])
    exit()


# add specified columns from the data file to global x1 axis in an X vs Y scatter
def addfileXY(fname, cols): 
    global ax1, ax2
    dcn = [] 
    for x in map(lambda i: colnames.index(i), cols):
        dcn += [x, ]

    array  = np.loadtxt(fname, usecols=dcn)
    ax1.plot(array[:, 0], array[:, 1], label=cols[0] + " vs " + cols[1])

# given an input file, calculate the output file name and make sure it exists
def makeFile(ifile, replay, git, dargs):
    if re.match("^.*\.plog$", ifile):
        ofile = ifile;
    else:
        if (re.match("^[0-9]{3}$", ifile)):
            ifile = "./logs/AHRSD" + ifile + ".DAT"
        m = re.match("^(.+)\.DAT$", ifile)
        ofile = (m[1] + ("-R" , "")[replay == 0] + 
            ("-GIT" + git, "")[len(git) == 0] + 
            ("-ARGS" + dargs, "")[len(dargs) == 0] + ".plog")

        if git != "":
            gitdir="/tmp/loglook.sh/" + git
            os.system("mkdir -p " + gitdir)
            os.system("pwd")
            os.system("git archive %s| tar -x -C '%s'" % (git, gitdir))
            os.system("(cd '%s' && sed -i 's/include /#/' Makefile && make winglevlr_ubuntu)" % gitdir)
            prog = gitdir + "/winglevlr_ubuntu"
        else: 
            prog = "./winglevlr_ubuntu"
            os.system("make " + prog)

        if not os.path.exists(ofile) or os.path.getctime(ofile) < os.path.getctime(prog):
            cmd = prog + " " + dargs + " --replay " + ifile + " --log " + ("-", "+")[replay]
            cmd += "| egrep '(LOG U$)|(LOG$)' > '" + ofile + "'"
            print(cmd)
            os.system(cmd)

    return ofile 

columns = []
diffArray = []

# adds the accumulated columns[] from filename to the axes
def run(filename, replay, git, dargs, xyPlot=False):
    global columns, diffArray
    if len(columns) > 0:
        ofile = makeFile(filename, replay, git, dargs)
        if not xyPlot:
            addfile(ofile, columns, diffArray)
        else:
            addfileXY(ofile, columns)
        columns = []
        diffArray = []


def runXY(filename, replay, git, dargs, cols):
    global columns
    run(filename, replay, git, dargs)
    columns = cols        
    run(filename, replay, git, dargs, xyPlot=True)


# load the diffArray with values to be used in subsequent plot
def diff(filename, replay, git, dargs, col):
    global diffArray 
    f = makeFile(filename, replay, git, dargs)
    diffArray = np.loadtxt(f, usecols=[colnames.index(col),])

#globals
mpl.rcParams["lines.linewidth"] = 1.5
fig, ax1 = plt.subplots()
ax2 = ax1.twinx()
ax2._get_lines.prop_cycler = ax1._get_lines.prop_cycler
filename = "./out.plog"
replay = 0 
git = cmdargs = ""
argv.pop(0)

# Parse arguments
while(len(argv) > 0):
    a = argv.pop(0)
    m = re.match("^[-]*([xy2]+)range=([-+]?[0-9.]+),([-+]?[0-9.]+)", a)
    if (m):   # handle -xrange, -yrange, -y2range=xxx,xxx  
        if m[1] == "x": plt.xlim([m[2], m[3]])
        if m[1] == "y": ax1.set_ylim(m[2], m[3])
        if m[1] == "y2": ax2.set_ylim(m[2], m[3])
    elif re.match("^[-]*args", a):
        run(filename, replay, git, cmdargs)
        cmdargs = argv.pop(0)
    elif re.match("^[-]*replay", a) or re.match("^[0-9]{3}$", a):
        # <--replay> 044 or --replay ./logs/AHRSD044.DAT 
        run(filename, replay, git, cmdargs)
        if re.match("^[-]*replay", a):
            filename = argv.pop(0)
        else:   
            filename = a
        replay = 1
        git = ""
        cmdargs = ""
    elif re.match("^[-]*reuse", a) or re.match(".*\.plog", a):
        # <--reuse> ./logs/AHRSD044.plog 
        run(filename, replay, git, cmdargs)
        if re.match("^[-]*reuse", a):
            filename = argv.pop(0)
        else: 
            filename = a;
        replay = 0
        git = ""
        cmdargs = ""
    elif re.match("^[-]*git", a):
        run(filename, replay, git, cmdargs)
        replay = 1
        git = argv.pop(0)
    elif re.match("^[-]*diff", a):
        run(filename, replay, git, cmdargs)
        diff(filename, replay, git, cmdargs, argv.pop(0))
    elif re.match("^[-]*run", a):
        run(filename, replay, git, cmdargs)
    elif re.match("^[-]*map", a):
        runXY(filename, replay, git, cmdargs, ["lon", "lat"]);
    elif re.match("^[-]*magcal", a):
        run(filename, replay, git, cmdargs)
        magCalibrate(filename);
    elif re.match("^[-]*magplot", a):
        runXY(filename, replay, git, cmdargs, ["mx", "my"]);
    else:
        columns += [a, ]
        if (len(diffArray) > 0):
            run(filename, replay, git, cmdargs)

if len(columns)> 0:
    run(filename, replay, git, cmdargs)

for a in [ax1, ax2]:
    a.margins(0,0)
    a.legend()
ax1.grid(b=True, axis="both", alpha=.3) 
plt.get_current_fig_manager().full_screen_toggle()
plt.subplots_adjust(left=0.03, right=0.97, top=0.97, bottom=0.03)
plt.show(block=True)
