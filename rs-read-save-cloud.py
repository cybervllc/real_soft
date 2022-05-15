import pyrealsense2 as rs
import numpy as np
import cv2
import pcl
import os
import matplotlib.pyplot as plt
import open3d as o3d
import json
import subprocess
import yaml
import rosbag
from cv_bridge import CvBridge
from pyntcloud import PyntCloud     

#------------------------------------------------------------------
def list_devices():
    #cписок устройств типа RealSense
    print ('Вывод характеристик устройств...')
    o3d.t.io.RealSenseSensor.list_devices()
    print ('Завершено ... вывод характеристик устройств...')
#------------------------------------------------------------------
 
#------------------------------------------------------------------ 
def save_color_depth_info_images(fpath, fname):
    # Конфигурация глубины и формата записи
    print ('Запись изображений RGB, Depth, Depth_Info....')
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Запуск стриминга с карты
    pipeline.start(config)
     # Получение изображений с RealSense. 10 фреймов
    for i in range(150):
        data = pipeline.wait_for_frames()
        depth = data.get_depth_frame()
        color = data.get_color_frame()
     # Получение профилей цветности и глубины у RealSense
    dprofile = depth.get_profile()
    cprofile = color.get_profile()
    cvsprofile = rs.video_stream_profile(cprofile)
    dvsprofile = rs.video_stream_profile(dprofile)
    color_intrin=cvsprofile.get_intrinsics()
    print(color_intrin)
    depth_intrin=dvsprofile.get_intrinsics()
    print(color_intrin)
    extrin = dprofile.get_extrinsics_to(cprofile)
    print(extrin)
    depth_image = np.asanyarray(depth.get_data())
    color_image = np.asanyarray(color.get_data())
     # Цветовая карта на изображении с глубиной (с конвертацией в 8 пиксельный формат)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    cv2.imwrite(fpath + '/color/' + fname +'.png', color_image)
    cv2.imwrite(fpath + '/depth/' + fname +'.png', depth_image)
    cv2.imwrite(fpath + '/depth_info/' + fname + '.png', depth_colormap)
    print('Изображения RGB, Depth, Depth_Info сохранены...')
    pipeline.stop()
#-----------------------------------------------------------------------

#-----------------------------------------------------------------------    
def save_txtcloud(fpath, fname):
    print ('Запись изображений в текстовый формат облака точек....')
    pc = rs.pointcloud()
    points = rs.points()
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipe_profile = pipeline.start(config)
    for i in range(150):
        data = pipeline.wait_for_frames()
        depth = data.get_depth_frame()
        color = data.get_color_frame()
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    color = frames.get_color_frame()
    colorful = np.asanyarray(color.get_data())
    colorful=colorful.reshape(-1,3)
    pc.map_to(color)
    points = pc.calculate(depth)
    # Определение координат вершин
    vtx = np.asanyarray(points.get_vertices())
    # Определение координат текстур
    # tex = np.asanyarray(points.get_texture_coordinates())
    with open(fpath + '/depth_info/' + fname + '.txt','w') as f:
        for i in range(len(vtx)):
            f.write(str(np.float(vtx[i][0])*1000)+' '+str(np.float(vtx[i][1])*1000)+' '+str(np.float(vtx[i][2])*1000)+' '+str(np.float(colorful[i][0]))+' '+str(np.float(colorful[i][1]))+' '+str(np.float(colorful[i][2]))+'\n')
    print ('Завершено .... запись изображений в текстовый формат облака точек....')
#----------------------------------------------------------------------------

#----------------------------------------------------------------------------
def set_log_dir(name): #каталог результатов и логов,  должен быть подготовлен с названием ~/name
    print ('Инициализация каталогов результатов....')
    log_dir = os.path.join(os.path.normpath(os.getcwd() + os.sep + os.pardir), name)
    if  not os.path.exists(log_dir):
        os.makedirs(log_dir)
        os.makedirs(log_dir + '/bag')
        os.makedirs(log_dir + '/color')
        os.makedirs(log_dir + '/depth')
        os.makedirs(log_dir + '/depth_info')
        os.makedirs(log_dir + '/ply')
        print('Каталоги для результатов инициализированы...')
    return log_dir
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
def show_color_depth_info_png(fpath, fname):
    cimages = fpath+'/color/'+fname+'.png'
    cimages = cv2.imread(cimages)
    dimages = fpath+'/depth/'+fname+'.png'
    dimages = cv2.imread(dimages)
    diimages = fpath+'/depth_info/'+fname+'.png'
    diimages = cv2.imread(diimages)
    plt.figure()
    f, ax = plt.subplots(1,3)
    ax[0].imshow(cimages)
    ax[1].imshow(dimages)
    ax[2].imshow(diimages)
    plt.show(block=True)
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
def save_bag(fpath, fname):
    print ('Запись в bag файл...')
    with open(fpath + '/config.json') as cf:
       rs_cfg = o3d.t.io.RealSenseSensorConfig(json.load(cf)) # Чтение конфигурации RealSense
 
    rs = o3d.t.io.RealSenseSensor()
    rs.init_sensor(rs_cfg, 0, fpath + '/bag/'+ fname + '.bag')
    rs.start_capture(True)  # true: Старт записи
    for fid in range(150):
         im_rgbd = rs.capture_frame(True, True)  # Захват фреймов
    # Здесь обрабатывам im_rgbd.depth and im_rgbd.color
    rs.stop_capture() # Стоп запись
    print ('Завершено ... запись в bag файл...')
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
def rosbag_to_images(fpath, fname):
    print('Извлечение изображений из bag файла....')
    FILENAME = fname
    ROOT_DIR = fpath
    BAGFILE = fpath + '/bag/'+ fname + '.bag'
    bag = rosbag.Bag(BAGFILE)
    for i in range(2):
        if (i == 0):
            TOPIC = '/device_0/sensor_0/Depth_0/image/data/'
            DESCRIPTION = 'depth_'
        else:
            TOPIC = '/device_0/sensor_1/Color_0/image/data'
            DESCRIPTION = 'color_'
        image_topic = bag.read_messages(TOPIC)
        for k, b in enumerate(image_topic):
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(b.message, b.message.encoding)
            cv_image.astype(np.uint8)
            if (DESCRIPTION == 'depth_'):
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imwrite(ROOT_DIR + '/depth/' + DESCRIPTION + str(b.timestamp) + '.png', cv_image)
            else:
                cv2.imwrite(ROOT_DIR + '/color/' + DESCRIPTION + str(b.timestamp) + '.png', cv_image)
            print('Сохранено: ' + DESCRIPTION + str(b.timestamp) + '.png')
    bag.close()
    print('Завершено извлечение изображений из bag файла....')
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
def save_to_ply(fpath, fname):
    print('Записб изображения в ply формат.....')
    pipe = rs.pipeline()
    cfg = rs.config()
    #cfg.enable_device_from_file("/home/realsense/real_soft/tst.bag")
    profile = pipe.start(cfg)

    for x in range(10):
        pipe.wait_for_frames()
    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()
    pipe.stop()
    color = np.asanyarray(color_frame.get_data())
    plt.rcParams["axes.grid"] = False
    plt.imshow(color)
    colorizer = rs.colorizer()
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    plt.imshow(colorized_depth)
    pc = rs.pointcloud();
    pc.map_to(color_frame);
    pointcloud = pc.calculate(depth_frame);
    pointcloud.export_to_ply(fpath + '/ply/'+ fname + '.ply', color_frame);
    cloud = PyntCloud.from_file(fpath + '/ply/'+ fname + '.ply');
    cloud.plot()
    print('Завершено .... запись изображения в ply формат.....')
#--------------------------------------------------------------------------------

if __name__ == "__main__":
    # Установка каталога для логов и результатов
    fpath = set_log_dir('logs') 
    print('Установлен каталог логов и результатов...: ' + fpath)

    list_devices()

    # Установка имени файла для записи изображений с RealSense
    fname = 'img1'
    
    # Запись изображений с цветом, глубиной и информацией о глубине. Время записи по умолчанию 10 кадров
    #save_color_depth_info_images(fpath, fname)
    
    #Запись информации в   txt облако точек
    #save_txtcloud(fpath, fname)

    # Запись изображений в ply формат
    #save_to_ply(fpath, fname)

    #save_bag(fpath,fname)

    rosbag_to_images(fpath,fname)
