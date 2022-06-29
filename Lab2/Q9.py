# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

#in mm
sensor_width = 13.22
sensor_heigth = 8.8

im_pixel_w = 5472
im_pixel_h = 3648

#in mm
f=8.8

#in cm/px
pix_res_1 = 0.69
pix_res_2 = 1.37
pix_res_3 = 2.06
pix_res_4 = 2.74

fly_dis = 20*60*5

W = 400
H = 200

def find_alt(pix_res):

    alt_w = pix_res*f*im_pixel_w/sensor_width/100

    alt_h = pix_res*f*im_pixel_h/sensor_width/100
    
    if alt_w > alt_h:
        alt = alt_w
    else:
        alt = alt_h
        
    return alt

def cal_side_lap(pix_res):
    
    cam_ftpt_w = pix_res*im_pixel_w/100
    cam_ftpt_h = pix_res*im_pixel_h/100
    
    if cam_ftpt_w > cam_ftpt_h:
        width = cam_ftpt_w
    else:
        width = cam_ftpt_h
        
    #fly along 400m
    line_num = int((fly_dis - H)/W)
    side_lap_W = (line_num*width - H)/((line_num - 1)*width)
    
    #fly along 200m
    line_num = int((fly_dis - W)/H)
    side_lap_H = (line_num*width - W)/((line_num - 1)*width)
    
    return side_lap_W, side_lap_H

'''
altitude_1 = find_alt(pix_res_1)
altitude_2 = find_alt(pix_res_2)
altitude_3 = find_alt(pix_res_3)
altitude_4 = find_alt(pix_res_4)


print(altitude_1)
print(altitude_2)
print(altitude_3)
print(altitude_4)
'''

#fly along 400m
side_lap_1_W, side_lap_1_H = cal_side_lap(pix_res_1)
side_lap_2_W, side_lap_2_H = cal_side_lap(pix_res_2)
side_lap_3_W, side_lap_3_H = cal_side_lap(pix_res_3)
side_lap_4_W, side_lap_4_H = cal_side_lap(pix_res_4)

print(side_lap_1_W)
print(side_lap_2_W)
print(side_lap_3_W)
print(side_lap_4_W)
print(side_lap_1_H)
print(side_lap_2_H)
print(side_lap_3_H)
print(side_lap_4_H)

    
    



