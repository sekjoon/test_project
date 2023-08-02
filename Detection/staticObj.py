#!/usr/bin/env python
from std_msgs.msg import Int32MultiArray, Bool, Int32
from sensor_msgs.msg import LaserScan, CompressedImage
from cv_bridge import CvBridge

import rospy
import cv2
import math

class Static:
    def __init__(self):
        rospy.init_node('Static')
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.origin = LaserScan() # 라이다 데이터 전역 변수
        self.modify = rospy.Publisher('/check', LaserScan, queue_size=10)
        self.static_pub = rospy.Publisher('/staticCase', Int32, queue_size=1)
        self.low_threshold = 10 # 정적 물체 포인트 개수
        self.high_threshold = 100 # 정적 물체 포인트 개수
        self.staticThreshold = 0.7 # 정적 물체 인식 거리
        self.farobjThreshold = 0.5 # 물체에서 일정 거리보다 멀면 inf 처리
        self.step_one = True
        self.step_two = False
        self.step_thr = False
        # second detection condition desired
        
    
    def lidar_callback(self, data):
        # resolution : 1285
        # increment : 0.28 deg
        staticOn = Int32()
        staticOn.data = 0
        self.origin = data

        modified_ranges = list(data.ranges)[:] # 얕은 복사, 전방 시야 가공 데이터
        origin_ranges = list(data.ranges)[:] # 얕은 복사, 원본 데이터
        # data modifying
        for i in range(len(list(data.ranges))):
            if 108<=i<=1177: # front ROI. 0~56 deg, 310 ~ 360 deg
                modified_ranges[i] = math.inf
        
        data.ranges = tuple(modified_ranges)
        self.modify.publish(data) # 가공데이터 publish
        
        data.ranges = tuple(origin_ranges) # 원상복귀
        frontMin = min(modified_ranges) # 전방에서 가장 가까운 거리
        aroundMin = min(data.ranges) # 360도 중 가장 가까운 거리

        cnt = 0
        # compare with minimum distance and around obj distance
        for i in modified_ranges:
            if abs(frontMin-i) <= self.farobjThreshold:  
                cnt += 1

        # print(id(origin_ranges))
        # print(id(modified_ranges))
        print(f'frontMin : {frontMin}, idx : {modified_ranges.index(frontMin)}')
        print(f'arundMin : {aroundMin}, idx : {origin_ranges.index(aroundMin)}')
        print(f'cnt : {cnt}')

        if self.step_one == True and self.low_threshold <= cnt <= self.high_threshold:
            if frontMin <= self.staticThreshold:
                print(11111111111111111)
                staticOn.data = 1
                self.static_pub.publish(staticOn)
                self.step_one = False
                self.step_two = True

        if self.step_two == True and 160 <= data.ranges.index(aroundMin) <= 400 or 1114 <= data.ranges.index(aroundMin) <= 1124:
            if frontMin<= 0.6:
                print(2222222222222222)
                staticOn.data = 2
                self.static_pub.publish(staticOn)
                self.step_two = False
                self.step_thr = True

        if self.step_thr == True and ~~~     
        
        # elif staticOn==2 and 
             
        
        
        
        
        
        
        
        # print(f'range : {ranges[350:190]}')
        

        # print(ranges)

    # def 


if __name__ == '__main__':
    Static()
    rospy.spin()
