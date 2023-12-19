#!/usr/bin/env python3
# import rospy
import rclpy
from sensor_msgs.msg import Image, CompressedImage
from rclpy.node import Node
from threading import Thread
from time import sleep, ctime
import numpy as np
import queue
import time
import cv2
from functools import partial
from rtsp_img_publish.rtsp_cam import RTSPCam
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge
from hbm_img_msgs.msg import HbmMsg1080P
from array import array
class RTSPImgNode(Node):
    def __init__(self, name):
      super().__init__(name)
      self.param_init()
      self.br = CvBridge()
      chn = self.start_decode_chn
      self.timer_list = []
      self.cam_list = []
      for cam_config in self.cam_config_list:
        img_format = cam_config["img_format"]
        topic_name = '/pic/' + cam_config["name"] + '_' + img_format
        rtsp_url = cam_config["url"]
        if img_format.find('nv12')!=-1:
          rtsp_cam = RTSPCam(rtsp_url, False, chn, self.get_logger())
          publisher_ = self.create_publisher(UInt8MultiArray, topic_name, self.img_topic_queue_size)
        else:
          rtsp_cam = RTSPCam(rtsp_url, True, chn, self.get_logger())
          publisher_ = self.create_publisher(Image, topic_name, self.img_topic_queue_size)
        rtsp_cam.start()
        call_back = partial(self.timer_callback, rtsp_cam, publisher_)
        timer = self.create_timer(0.1, call_back)
        self.timer_list.append(timer)
        self.cam_list.append(rtsp_cam)
        chn += 1
  
    def timer_callback(self, fpb, publisher):
      frame = fpb.get_frame()
      if frame is not None:  
        is_nv12 = fpb.is_nv12()
        if is_nv12:
           img = UInt8MultiArray( data = array('B', frame)) 
        else:
           img = self.br.cv2_to_imgmsg(frame)
        publisher.publish(img)
  
    def param_init(self):
      self.declare_parameter('img_topic_queue_size')
      self.img_topic_queue_size = self.get_parameter('img_topic_queue_size').get_parameter_value().integer_value
      self.declare_parameter('start_decode_chn')
      self.start_decode_chn = self.get_parameter('start_decode_chn').get_parameter_value().integer_value

      self.declare_parameter('rtsp_cam_num')
      size = self.get_parameter('rtsp_cam_num').get_parameter_value().integer_value
      cam_name_list = []
      for i in range(size):
        str = 'rtsp_cam%d' % (i + 1)
        cam_name_list.append(str)
      self.cam_config_list = []
      for cam_name in cam_name_list:
        cam_config = {}
        cam_name_key = cam_name + '.name'
        cam_url_key = cam_name + '.url'
        cam_imgformat_key = cam_name + '.img_format'
        self.declare_parameter(cam_name_key)
        self.declare_parameter(cam_url_key)
        self.declare_parameter(cam_imgformat_key)
        cam_name_value = self.get_parameter(cam_name_key).get_parameter_value().string_value
        cam_url_value = self.get_parameter(cam_url_key).get_parameter_value().string_value
        cam_imgformat_value = self.get_parameter(cam_imgformat_key).get_parameter_value().string_value
        cam_config["name"] = cam_name_value
        cam_config["url"] = cam_url_value
        cam_config["img_format"] = cam_imgformat_value
        self.cam_config_list.append(cam_config)
    def stop(self):
        for cam in self.cam_list:
          cam.cam_stop()
          cam.join()


def main(args=None):
    rclpy.init(args=args)
    RTSP_node = RTSPImgNode("rtsp_node")
    try:
      rclpy.spin(RTSP_node)
    except:
      RTSP_node.stop()
    RTSP_node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
