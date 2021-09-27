#!/usr/bin/python3

# Plot the files "out.plog" and "wpts.txt" on google maps, output file to "map.html"
# Display with "google-chrome ./map.html"

import gmplot
import re 

apikey = 'AIzaSyC4_GZpzLJsbb_XgHka26mQQTa-QaO9d3w'
gmap = gmplot.GoogleMapPlotter(47.456185024831434, -122.4923591060998, 13, apikey=apikey)

track = []
waypoints = []
lcount = 0

with open('./out.plog') as f:
    line = f.readline()
    while line:
        line = f.readline()
        lcount += 1
        if (lcount % 100 == 0):           
            words = re.split("\s+", line)
            if (len(words) > 51):
                lat = float(words[50])
                lon = float(words[51])
                track.append((lat, lon))


with open('./wpts.txt') as f:
    line = f.readline()
    while line:
        line = f.readline()
        m = re.match("\s*([\d.+-]+)\s*,\s*([\d.+-]+)", line)            
        if (m):
            waypoints.append((float(m.group(1)), float(m.group(2))))

waypoints.append(waypoints[0])

gmap.plot(*zip(*waypoints), edge_width=8, color='green')
gmap.plot(*zip(*track), edge_width=2, color='red')
gmap.draw('map.html')

