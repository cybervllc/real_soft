import open3d as o3d
import os
import json


#cписок устройств типа RealSense
o3d.t.io.RealSenseSensor.list_devices()


#
fileDir = os.path.dirname(os.path.realpath(__file__))
print (fileDir)

with open('/home/realsense/real_soft/config.json') as cf:
    rs_cfg = o3d.t.io.RealSenseSensorConfig(json.load(cf)) # Чтение конфигурации RealSense

rs = o3d.t.io.RealSenseSensor()
rs.init_sensor(rs_cfg, 0, '/home/realsense/real_soft/tst.bag')
rs.start_capture(True)  # true: Старт записи
for fid in range(150):
    im_rgbd = rs.capture_frame(True, True)  # Захват фреймов
    # Здесь обрабатывам im_rgbd.depth and im_rgbd.color

rs.stop_capture() # Стоп запись