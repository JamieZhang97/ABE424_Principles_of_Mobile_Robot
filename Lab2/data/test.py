# -*- coding: utf-8 -*-
"""
Created on Tue Mar 30 17:44:14 2021

@author: Jamie
"""

import json
import pyproj
import matplotlib.pyplot as plt

with open('autonomous-farm.geojson', 'r') as f:
    data = json.load(f)
GPS=data['features'][0]['geometry']['coordinates'][0]

x = []
y = []

proj = pyproj.Proj(proj='utm',ellps='WGS84')
x_0, y_0 = proj(GPS[0][0], GPS[0][1])

for i in range(7):
    dx, dy = proj(GPS[i][0], GPS[i][1])

    x.append(dx - x_0)
    y.append(dy - y_0)

plt.plot(x, y, '-o')