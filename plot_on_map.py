#!/usr/bin/python3

# Plot the files "out.plog" and "wpts.txt" on google maps, output file to "map.html"
# Display with "google-chrome ./map.html"

import gmplot
import re 
import sys

filePlog = "./out.plog" if len(sys.argv) < 2 else sys.argv[1]
fileWpts = "./wpts.txt" if len(sys.argv) < 3 else sys.argv[2]



apikey = 'AIzaSyC4_GZpzLJsbb_XgHka26mQQTa-QaO9d3w'
gmap = gmplot.GoogleMapPlotter(47.456185024831434, -122.4923591060998, 13, apikey=apikey)

track = []
waypoints = []
lcount = 0

with open(filePlog) as f:
    line = f.readline()
    while line:
        lcount += 1
        if (lcount % 100 == 0):           
            words = re.split("\s+", line)
            if (len(words) > 30):
                lat = float(words[29])
                lon = float(words[30])
                track.append((lat, lon))
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

gmap.scatter(*zip(*waypoints), color='yellow')
gmap.plot(*zip(*track), edge_width=3, color='red')
gmap.draw('map.html')

