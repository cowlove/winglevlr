#!/usr/bin/python3

# Plot the files "out.plog" and "wpts.txt" on google maps, output file to "map.html"
# Display with "google-chrome ./map.html"

import gmplot
import re 
import sys

filePlog1 = "./out.plog" if len(sys.argv) < 2 else sys.argv[1]
filePlog2 = "./out.plog" if len(sys.argv) < 3 else sys.argv[2]
fileWpts = "./wpts.txt" if len(sys.argv) < 4 else sys.argv[3]



apikey = 'AIzaSyC4_GZpzLJsbb_XgHka26mQQTa-QaO9d3w'
gmap = gmplot.GoogleMapPlotter(47.490366884467925, -122.8146229910965, 13, apikey=apikey)
#47.45804489362055, -122.59075938584017, 13, apikey=apikey)

track1 = []
track2 = []
waypoints = []
lcount = 0

with open(filePlog1) as f:
    line = f.readline()
    while line:
        lcount += 1
        if (lcount % 100 == 0):           
            words = re.split("\s+", line)
            if (len(words) > 30):
                lat = float(words[30])
                lon = float(words[31])
                track1.append((lat, lon))
        line = f.readline()

with open(filePlog2) as f:
    line = f.readline()
    while line:
        lcount += 1
        if (lcount % 100 == 0):           
            words = re.split("\s+", line)
            if (len(words) > 30):
                lat = float(words[30])
                lon = float(words[31])
                track2.append((lat, lon))
        line = f.readline()


with open(fileWpts) as f:
    line = f.readline()
    while line:
        m = re.match("\s*([\d.+-]+)\s*,\s*([\d.+-]+)", line)            
        if (m):
            waypoints.append((float(m.group(1)), float(m.group(2))))
        line = f.readline()
    
if (len(waypoints) > 0):
    waypoints.append(waypoints[0])

gmap.scatter(*zip(*waypoints), color='blue')
gmap.plot(*zip(*track1), edge_width=1, color='yellow')
gmap.plot(*zip(*track2), edge_width=4, color='yellow')
gmap.draw('map.html')

