import threading
import queue
import rclpy
from threading import Thread
from time import sleep, ctime
import numpy as np
import queue
import time
import cv2
from rtsp_img_publish import tools
from hobot_vio import libsrcampy

class RTSPCam(threading.Thread):
  def __init__(self, rtsp_url, covert_to_bgr, decode_chn_num, logger_handle, max_queue_size = 3):
    threading.Thread.__init__(self)
    self.rtsp_url = rtsp_url
    self.covert_to_bgr = covert_to_bgr
    self.decode_chn_num = decode_chn_num  
    self.frame_queue = queue.Queue(maxsize = max_queue_size)
    self.logger_handle = logger_handle
    self.is_stop = False

  def is_nv12(self):
     return not self.covert_to_bgr

  def cam_stop(self):
      self.is_stop = True

  def get_resolution(self):
     return (self.height, self.width)

  def close_rtsp_and_decoder(self):
    self.find_pps_sps = 0
    self.skip_count = 0
    self.image_count = 0
    self.get_image_failed_count = 0
    self.decode_image_failed_count = 0
    self.start_time = time.time()
    self.dec.close()
    self.cap.release()

  def open_rtsp_and_decoder(self):
    while True:
      self.cap = cv2.VideoCapture(self.rtsp_url)
      if not self.cap.isOpened():
          self.logger_handle.warning("Open rtsp(%s) failed! Sleep 5sec" % (self.rtsp_url))
          sleep(5)
      else:
          self.logger_handle.info("Open rtsp(%s) OK!" % (self.rtsp_url))
          break

    self.cap.set(cv2.CAP_PROP_FORMAT, -1) # get stream
    self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    self.logger_handle.info("width :%d height: %d"%(self.width, self.height))

    self.dec = libsrcampy.Decoder()
    ret = self.dec.decode("", self.decode_chn_num, 1, self.width, self.height)
    self.logger_handle.info("Decoder return:%d frame count: %d" %(ret[0], ret[1]))
    if(ret[0]==-1):
        self.logger_handle.warning("Decoder open failed!")
        return False
    self.find_pps_sps = 0
    self.skip_count = 0
    self.image_count = 0
    self.get_image_failed_count = 0
    self.decode_image_failed_count = 0
    self.start_time = time.time()
    return True

  def reopen(self):
    self.close_rtsp_and_decoder()
    return self.open_rtsp_and_decoder()  

  def get_frame(self):
    if self.frame_queue.empty() != True:
      return self.frame_queue.get()
    else:
      return None

  def run(self):
    if self.open_rtsp_and_decoder() == False:
      self.close_rtsp_and_decoder()
      self.logger_handle.warning("Decoder open failed! Exit!")
      return

    while(self.is_stop == False):
      if not self.cap.isOpened():
        if self.reopen() == False:
           self.close_rtsp_and_decoder()
           self.logger_handle.warning("Reopen failed! Exit!")
           return  
      else:
        ret, frame = self.cap.read()
        if ret==0 or (frame is None): 
            if self.reopen() == False:
              self.close_rtsp_and_decoder()
              self.logger_handle.warning("Reopen failed! Exit!")
              return    
            continue
        if self.find_pps_sps == 0:
            if tools.is_pps_sps(frame.tobytes()) == False:
                continue
            else:
                self.find_pps_sps = 1
        ret = self.dec.set_img(frame.tobytes(), self.decode_chn_num)
        if ret !=0:
            self.logger_handle.warning("decode failed!")
            self.decode_image_failed_count += 1
            if(self.decode_image_failed_count >= 10):
              if self.reopen() == False:
                self.close_rtsp_and_decoder()
                self.logger_handle.warning("Reopen failed! Exit!")
                return
            continue
        else:
            self.decode_image_failed_count = 0
        if self.skip_count < 8:
            self.skip_count += 1
            continue
        nv12_frame = self.dec.get_img()
        if nv12_frame is not None:
            self.get_image_failed_count = 0
            if self.covert_to_bgr == True:
              img = tools.nv12_bgr(nv12_frame, self.width, self.height)
            else:
               img = nv12_frame
            if self.frame_queue.full() == False:
              self.frame_queue.put(img)
        else:
          self.get_image_failed_count += 1
          self.logger_handle.warning("Get frame failed!")
          if(self.get_image_failed_count >= 10):
            if self.reopen() == False:
              self.close_rtsp_and_decoder()
              self.logger_handle.warning("Reopen failed! Exit!")
              return
        finish_time = time.time()
        self.image_count += 1
        if finish_time - self.start_time > 30:
            self.logger_handle.info('Decode CHAN: %.2f' % (self.image_count / (finish_time - self.start_time)))
            self.start_time = finish_time
            self.image_count = 0
    self.close_rtsp_and_decoder()
    self.logger_handle.warning("end!")