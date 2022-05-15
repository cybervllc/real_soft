import cv2                                
import numpy as np                       
import matplotlib.pyplot as plt          
from pyntcloud import PyntCloud          
import pyrealsense2 as rs                



pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("/home/realsense/real_soft/tst.bag")
profile = pipe.start(cfg)


for x in range(5):
  pipe.wait_for_frames()
  

frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
depth_frame = frameset.get_depth_frame()


pipe.stop()
print("Frames Captured")

color = np.asanyarray(color_frame.get_data())
plt.rcParams["axes.grid"] = False
plt.imshow(color)



colorizer = rs.colorizer()
colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
plt.imshow(colorized_depth)

pc = rs.pointcloud();
pc.map_to(color_frame);
pointcloud = pc.calculate(depth_frame);
pointcloud.export_to_ply("/home/realsense/real_soft/tst.ply", color_frame);
cloud = PyntCloud.from_file("/home/realsense/real_soft/tst.ply");
cloud.plot()
