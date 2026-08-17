'''
Author: 柒上夏OPO
Date: 2022-01-10 12:31:28
LastEditTime: 2022-01-14 14:51:00
LastEditors: CloudSir
Description: 
'''

import numpy as np
import cv2
from socket import *
from matplotlib import pyplot as plt

# 127.0.0.1表示本机的IP，用于测试，使用时需要改为服务端的ip
addr = ('127.0.0.1', 8081) 

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture("/home/zzx/vid.mp4")

# 设置镜头分辨率，默认是640x480
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

s = socket(AF_INET, SOCK_DGRAM) # 创建UDP套接字
# s = socket(AF_INET, SOCK_STREAM) # 创建TCP套接字
cv2.namedWindow('client',cv2.WINDOW_NORMAL)
# cv2.WINDOW_NORMAL：窗口大小可以调整。
# cv2.WINDOW_AUTOSIZE：窗口大小根据图像大小自动调整。
# cv2.WINDOW_FULLSCREEN：窗口全屏显示。
# cv2.WINDOW_FREERATIO：窗口大小可以自由调整，但图像的宽高比不变。
# cv2.WINDOW_KEEPRATIO：窗口大小可以自由调整，但图像的宽高比保持不变。
cap.set(cv2.CAP_PROP_FPS, 120)
fps = int(cap.get(cv2.CAP_PROP_FPS))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("fps=========== ",fps, width, height)

while True:
    _, img = cap.read()

    img = cv2.flip(img, 1)

    # img=cv2.resize(img,(640,480),interpolation=cv2.INTER_CUBIC)

    # 压缩图片
    _, send_data = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 30])

    print(f'正在发送数据，大小:{send_data.size} Byte')

    s.sendto(send_data, addr)


    cv2.putText(img, "client", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    cv2.imshow('client', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

s.close()
cv2.destroyAllWindows()

