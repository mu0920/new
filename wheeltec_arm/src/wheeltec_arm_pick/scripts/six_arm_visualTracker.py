#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import rospy
import numpy as np
import cv2
from std_msgs.msg import Int8
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
from wheeltec_arm_pick.msg import six_arm_position as PositionMsg
from dynamic_reconfigure.server import Server
from wheeltec_arm_pick.cfg import Params_colorConfig

# 全局标志变量
choose_flag = 1 
trackstart = 0
m = 1  # 默认选择蓝色

class VisualTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.i = 0
        self.last_mask = None
        self.opencv_version = cv2.__version__.split('.')[0]
        rospy.loginfo("使用 OpenCV 版本: %s", self.opencv_version)
        
        # 默认颜色阈值 (HSV格式)
        self.default_colors = {
            'yellow': {'lower': [20, 100, 100], 'upper': [30, 255, 255]},
            'blue': {'lower': [90, 100, 50], 'upper': [130, 255, 255]},
            'green': {'lower': [40, 70, 50], 'upper': [80, 255, 255]},
            'user': {'lower': [77, 52, 5], 'upper': [165, 255, 255]}  # 更新默认值
        }
        
        # 初始化颜色范围
        self.color_ranges = {
            0: (np.array(self.default_colors['yellow']['lower']), 
                 np.array(self.default_colors['yellow']['upper']), 'yellow'),
            1: (np.array(self.default_colors['blue']['lower']), 
                 np.array(self.default_colors['blue']['upper']), 'blue'),
            2: (np.array(self.default_colors['green']['lower']), 
                 np.array(self.default_colors['green']['upper']), 'green'),
            3: (np.array(self.default_colors['user']['lower']), 
                 np.array(self.default_colors['user']['upper']), 'user-defined')
        }
        
        # 图像参数
        self.pictureHeight = rospy.get_param('~pictureDimensions/pictureHeight', 480)
        self.pictureWidth = rospy.get_param('~pictureDimensions/pictureWidth', 640)
        vertAngle = rospy.get_param('~pictureDimensions/verticalAngle', 0.785)  # 45度
        horizontalAngle = rospy.get_param('~pictureDimensions/horizontalAngle', 1.047)  # 60度
        
        self.tanVertical = np.tan(vertAngle)
        self.tanHorizontal = np.tan(horizontalAngle)
        
        # 形态学操作核
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        
        # 话题订阅发布
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.trackObject)
        self.positionPublisher = rospy.Publisher('/object_tracker/current_position', PositionMsg, queue_size=1)
        self.color_sub = rospy.Subscriber("/color_flag", Int8, self.colorflag_callback)
        self.visualflagPublisher = rospy.Publisher('/visual_clamp_flag', Int8, queue_size=1)
        
        # rqt在线调参服务
        self.color_obj = Server(Params_colorConfig, self.colorreconfigure)
        
        rospy.loginfo("视觉追踪器初始化完成")

    def publish_flag(self):
        visual_clamp_flag = Int8()
        visual_clamp_flag.data = 1
        self.visualflagPublisher.publish(visual_clamp_flag)

    def colorflag_callback(self, msg):
        global choose_flag
        choose_flag = msg.data

    def trackObject(self, image_data):
        global trackstart, m
        
        try:
            # 转换图像
            frame = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
            
            # 优化图像处理：只处理中心区域
            h, w = frame.shape[:2]
            roi = frame[int(h*0.1):int(h*0.9), int(w*0.1):int(w*0.9)]
            
            # 转换为HSV
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # 简化初始化处理
            if self.i < 2:
                self.i += 1
            elif self.i == 2:
                self.publish_flag()
                self.i = 3
            
            trackstart = 1
            
            # 获取当前颜色范围
            lower, upper, obj_color = self.color_ranges[m]
            
            # 创建掩码
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.dilate(mask, self.kernel, iterations=1)
            
            # 保存最后使用的掩码用于显示
            self.last_mask = mask
            
            # 处理不同OpenCV版本的findContours返回值差异
            if self.opencv_version == '3':
                _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            else:  # OpenCV 4.x
                contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # 只处理最大轮廓
                max_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(max_contour)
                
                if area > 200:
                    # 计算最小外接矩形
                    rect = cv2.minAreaRect(max_contour)
                    centerRaw = rect[0]
                    
                    # 计算角度
                    angleX = self.calculateAngleX(centerRaw)
                    angleY = self.calculateAngleY(centerRaw)
                    
                    # 发布位置
                    self.publishPosition(angleX, angleY, True, obj_color)
                    return
        
            # 未检测到物体
            self.publishPosition(0, 0, False, obj_color)
            
        except Exception as e:
            rospy.logerr("处理图像时出错: %s", str(e))
    
    def publishPosition(self, angleX, angleY, correct, color):
        try:
            posMsg = PositionMsg()
            posMsg.angleX = angleX
            posMsg.angleY = angleY
            posMsg.correct = correct
            posMsg.color = color
            self.positionPublisher.publish(posMsg)
        except rospy.ROSException as e:
            if "publish() to closed topic" not in str(e):
                rospy.logwarn("发布位置时出错: %s", str(e))
    
    def calculateAngleX(self, pos):
        displacement = 2 * pos[0] / self.pictureWidth - 1
        return -np.arctan(displacement * self.tanHorizontal)
    
    def calculateAngleY(self, pos):
        displacement = 2 * pos[1] / self.pictureHeight - 1
        return -np.arctan(displacement * self.tanVertical)
    
    def colorreconfigure(self, config, level):
        """动态重配置回调 - 更新所有颜色范围"""
        try:
            # 分别更新每种颜色的范围
            self.color_ranges[0] = (
                np.array([config.yellow_h_min, config.yellow_s_min, config.yellow_v_min]),
                np.array([config.yellow_h_max, config.yellow_s_max, config.yellow_v_max]),
                'yellow'
            )
            self.color_ranges[1] = (
                np.array([config.blue_h_min, config.blue_s_min, config.blue_v_min]),
                np.array([config.blue_h_max, config.blue_s_max, config.blue_v_max]),
                'blue'
            )
            self.color_ranges[2] = (
                np.array([config.green_h_min, config.green_s_min, config.green_v_min]),
                np.array([config.green_h_max, config.green_s_max, config.green_v_max]),
                'green'
            )
            self.color_ranges[3] = (
                np.array([config.user_h_min, config.user_s_min, config.user_v_min]),
                np.array([config.user_h_max, config.user_s_max, config.user_v_max]),
                'user-defined'
            )
            
            rospy.loginfo("成功更新所有颜色范围")
            
        except Exception as e:
            rospy.logerr("更新颜色配置时出错: %s", str(e))
        
        return config

def main():
    rospy.init_node('visual_tracker')
    tracker = VisualTracker()
    
    # 创建调参窗口
    cv2.namedWindow('Color Tracker', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('Color: 0=Yel,1=Blu,2=Grn,3=Usr', 
                      'Color Tracker', 1, 3, lambda x: None)
    
    rate = rospy.Rate(20)
    
    try:
        while not rospy.is_shutdown():
            global m
            m = cv2.getTrackbarPos('Color: 0=Yel,1=Blu,2=Grn,3=Usr', 'Color Tracker')
            
            # 显示处理结果
            if tracker.last_mask is not None:
                cv2.imshow("Color Tracker", tracker.last_mask)
                key = cv2.waitKey(1)
                if key == 27:  # ESC键退出
                    break
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()