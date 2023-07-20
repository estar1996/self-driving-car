#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError


# image_lane_bev 은 perspective view 형태의 이미지를 BEV(bird-eye-view) 형태로 전환하는 예제입니다.
# IPM (Inverse Perspective Mapping)
# Warping

# 노드 실행 순서
# 1. 이미지 warping 영역 지정
# 2. 원본 이미지 내 Warping 영역과 매칭 될 목적지 지점 정의
# 3. 원근 맵 행렬 생성 함수를 이용하여 매핑 좌표에 대한 원근 맵 행렬을 생성
# 4. 원근 맵 행렬에 대한 기하학적 변환

def warp_image(img, source_prop):
    img_size = (img.shape[1], img.shape[0])

    x = img.shape[1]
    y = img.shape[0]

    # TODO: (2) 원본 이미지 내 Warping 영역과 매칭 될 목적지 지점 정의
    '''
    destination_points = np.float32(
    
    source_points = source_prop * np.float32([[x, y]]* 4)
    '''
    destination_points = np.float32([
        [0, y],
        [0, 0],
        [x, 0],
        [x, y]
    ])

    source_points = source_prop * np.float32([[x, y]] * 4)

    # 이미지를 기하학적 변환을 해야합니다. 이미지를 인위적으로 확대, 축소, 위치 변경, 회전,
    # 왜곡을 하는 것으로 BEV는 원근이 있는 이미지를 원근을 없애는 작업을 해야합니다.
    # 기하학적 변환은 아핀변환과(Affine transformation) 원근변환(Perspective transformation)이 있습니다.
    # 우리는 원근 변환이라는 OpenCV를 사용합니다.

    # TODO: (3) 원근 맵 행렬 생성 함수를 이용하여 매핑 좌표에 대한 원근 맵 행렬을 생성합니다.
    '''
    perspective_transform = cv2.

    '''
    perspective_transform = cv2.getPerspectiveTransform(source_points, destination_points)
    # TODO: (4) 이후 원근 맵 행렬에 대한 기하학적 변환을 진행합니다.
    '''
    warped_img = cv2.

    '''
    warped_img = cv2.warpPerspective(img, perspective_transform, img_size)

    return warped_img, source_points


class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None
        # self.img_size = None
        # TODO: (1) 이미지 warping 영역 지정
        '''
        Bird's eye view 를 하기 위한 영역을 지정해야 합니다. 이지미 warping을 위해 영역을 비율로 만들어줘야 합니다.
        self.source_prop = np.float32(
        '''
        self.source_prop = np.float32([
            [0.01, 0.77],
            [0.45, 0.52],
            [0.535, 0.52],
            [1, 0.76]
        ])

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.img_size = self.img_bgr.shape
        except CvBridgeError as e:
            print(e)

        # TODO: (2-0) 이미지 Warping 시작
        img_warp, source_points = warp_image(self.img_bgr, self.source_prop)
        # cv2.circle(self.img_bgr, (source_points[0][0], source_points[0][1]), 10, (255, 0, 0), -1, cv2.LINE_AA)
        # cv2.circle(self.img_bgr, (source_points[1][0], source_points[1][1]), 10, (255, 0, 0), -1, cv2.LINE_AA)
        # cv2.circle(self.img_bgr, (source_points[2][0], source_points[2][1]), 10, (255, 0, 0), -1, cv2.LINE_AA)
        # cv2.circle(self.img_bgr, (source_points[3][0], source_points[3][1]), 10, (255, 0, 0), -1, cv2.LINE_AA)

        img_concat = np.concatenate([self.img_bgr, img_warp], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1)


def main():
    rospy.init_node('lane_birdview', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()


if __name__ == '__main__':
    main()
