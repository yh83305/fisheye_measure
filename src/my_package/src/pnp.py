#!/usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import mvsdk
from dscamera import DSCamera
from my_package.msg import direct
import os


def publish_camera_stream():
    # 初始化ROS节点
    rospy.init_node('camera_publisher', anonymous=True)
    # image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    # bridge = CvBridge()
    # 获取当前脚本文件的目录
    current_dir = os.path.dirname(__file__)
    # 构造相对路径
    relative_path = 'calibration.json'
    json_file = os.path.join(current_dir, relative_path)
    cam = DSCamera(json_file)
    print("start")

    # 打开相机
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        print("No camera was found!")
        return

    for i, DevInfo in enumerate(DevList):
        print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
    i = 0 if nDev == 1 else int(input("Select camera: "))
    DevInfo = DevList[i]
    print(DevInfo)

    # 打开相机
    hCamera = 0
    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        print("CameraInit Failed({}): {}".format(e.error_code, e.message))
        return

    # 获取相机特性描述
    cap = mvsdk.CameraGetCapability(hCamera)

    # 判断是黑白相机还是彩色相机
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

    # 黑白相机让ISP直接输出MONO数据，而不是扩展成R=G=B的24位灰度
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    # 相机模式切换成连续采集
    mvsdk.CameraSetTriggerMode(hCamera, 0)

    # 手动曝光，曝光时间30ms
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 10 * 1000)

    # 让SDK内部取图线程开始工作
    mvsdk.CameraPlay(hCamera)

    # 计算RGB buffer所需的大小，这里直接按照相机的最大分辨率来分配
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

    # 分配RGB buffer，用来存放ISP输出的图像
    # 备注：从相机传输到PC端的是RAW数据，在PC端通过软件ISP转为RGB数据（如果是黑白相机就不需要转换格式，但是ISP还有其它处理，所以也需要分配这个buffer）
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    # 设置相机帧率
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            # 此时图片已经存储在pFrameBuffer中，对于彩色相机pFrameBuffer=RGB数据，黑白相机pFrameBuffer=8位灰度数据
            # 把pFrameBuffer转换成opencv的图像格式以进行后续算法处理
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth,
                                   1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))

            frame = cv2.resize(frame, (1280, 1024), interpolation=cv2.INTER_LINEAR)

            result = find_white_circles_using_Contours(frame)
            if result is not None:
                unproj_pts, _ = cam.cam2world(result)
                point = direct()
                point.x = unproj_pts[0][0]
                point.y = unproj_pts[0][1]
                point.z = unproj_pts[0][2]

                rospy.loginfo(f"Publishing Point: x={point.x}, y={point.y}, z={point.z}")

            rate.sleep()

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))

    # 释放相机资源
    # 关闭相机
    mvsdk.CameraUnInit(hCamera)
    # 释放帧缓存
    mvsdk.CameraAlignFree(pFrameBuffer)
    cv2.destroyAllWindows()


def find_white_circles_using_Contours(img):
    # 应用阈值化
    _, thresh = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)

    # 找到二值图像中的轮廓
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 初始化最大轮廓
    max_contour = max(contours, key=cv2.contourArea, default=None)

    if max_contour is not None:
        # 计算最大轮廓的中心
        M = cv2.moments(max_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # 绘制圆心
            cv2.circle(thresh, (cX, cY), 5, (0, 0, 255), -1)

            # 绘制最大轮廓
            cv2.drawContours(thresh, [max_contour], -1, (0, 255, 0), 2)

            # 显示结果
            cv2.imshow("Detected Max Contour", thresh)
            cv2.waitKey(1)

            return [cX, cY]
        else:
            return None
    else:
        return None


if __name__ == '__main__':
    try:
        publish_camera_stream()
    except rospy.ROSInterruptException:
        pass
