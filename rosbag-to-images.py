import subprocess
import yaml
import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np


FILENAME = 'tst'
ROOT_DIR = '/home/realsense/real_soft/'
BAGFILE = ROOT_DIR + '/' + FILENAME + '.bag'

if __name__ == '__main__':
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
            print('saved: ' + DESCRIPTION + str(b.timestamp) + '.png')


    bag.close()

    print('PROCESS COMPLETE')