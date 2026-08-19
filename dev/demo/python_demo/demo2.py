#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time     : 2023/5/16 10:22
# @Author   : Chenan_Wang
# @File     : cv_test.py
# @Project  : pad_det 
# @Software : PyCharm

import cv2 as cv
print("version=",cv.__version__)

if __name__ == "__main__":
    cap = cv.VideoCapture(0)
    # cap.open(1, cv.CAP_DSHOW)       # 我这里0为电脑自带摄像头，1为外接相机
    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    # cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('H', '2', '6', '4'))

    # cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)     
    # cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv.CAP_PROP_FPS, 120)
    fps = int(cap.get(cv.CAP_PROP_FPS))
    width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    print("fps=========== ",fps, width, height)
    cv.namedWindow('cv_test',cv.WINDOW_NORMAL)

    while True:
        if not cap.isOpened():
            print('can not open camera')
            break
        ret, frame = cap.read()     # 读取图像
        if not ret:                 # 图像读取失败则直接进入下一次循环
            continue
        cv.imshow('cv_test', frame)
        my_key = cv.waitKey(1)
        # 按q退出循环，0xFF是为了排除一些功能键对q的ASCII码的影响
        if my_key & 0xFF == ord('q'):
            break

    #释放资源
    cap.release()
    cv.destroyAllWindows()