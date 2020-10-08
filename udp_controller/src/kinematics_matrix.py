#!/usr/bin/env python
# coding: utf-8
import numpy as np

px = 0.505
py = 0.510
d = (px+py)/2
inv_d = 1/d
r_w = 0.1016
gear_ratio = 15
rads2rpm=30/np.pi
driver_constant = rads2rpm * gear_ratio * 2**15/12000
encoder_constant=2*np.pi/4000/gear_ratio
encoder_counter_maxvalue=65536

jacobian = np.array([[-1,  -1,  d],
					 [-1,   1, -d],
					 [-1,   1,  d],
					 [-1,  -1, -d]], np.float32) / r_w

inv_jacobian = np.array([[   -1,    -1,    -1,     -1],
                         [   -1,     1,     1,     -1],
                         [inv_d, -inv_d, inv_d, -inv_d]], np.float32) * r_w / 4

#inv_jacobian = np.array([[   -1,     -1,      0,      0],
#                         [    0,      1,      0,     -1],
#                         [inv_d,      0,      0, -inv_d]], np.float32) * r_w / 2







