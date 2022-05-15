import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np

b = bagreader('/home/realsense/real_soft/test.bag')

print (b.topic_table)

timg = b.message_by_topic('/device_0/sensor_0/Depth_0/image/data')

print(timg)

msg_img = pd.read_csv(timg)

print(msg_img)