#!/usr/bin/env python3

import sys

from PIL import Image
from sensor_msgs.msg import Image as ROSImage

from ros_msg_transform import dict_to_ros_msg, fill_ros_message, ros_msg_to_dict

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("[*] Usage: ./ros_image.py <PNG image file path>")
    message = ROSImage()

    print("------------------<sensor_msgs/Image>-------------------")
    im = Image.open("Lenna.png")
    width, height = im.size

    message.height = height
    message.width = width
    message.step = width
    message.data = str(list(im.getdata()))

    print("From sensor_msgs/Image to dict...")
    _dict = ros_msg_to_dict(message)
    print("From dict to sensor_msgs/Image...")
    msg = dict_to_ros_msg("sensor_msgs/Image", _dict)
    print("Fill ROS Message instance of sensor_msgs/Image...")
    msg = fill_ros_message(message, _dict)
    print("-------------------------------------------------------")
